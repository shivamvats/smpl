////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2018, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#include <smpl/search/smhastar.h>

#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/key.h>

#include <smpl/console/console.h>
#include <smpl/time.h>

namespace smpl {

static const char* LOG = "search.smhastar";
static const char* ELOG = "search.smhastar.expansions";
static const char* SLOG = "search.smhastar.successors";
static const char* DLOG = "search.smhastar.expanded_state";

static double GetTime()
{
    return to_seconds(clock::now().time_since_epoch());
}

SMHAStar::SMHAStar() : SBPLPlanner()
{
    // Overwrite default members for ReplanParams to represent a single optimal
    // search
    m_params.initial_eps = 1.0;
    m_params.final_eps = 1.0;
    m_params.dec_eps = 0.2; // NOTE: same initial epsilon delta as ARA*
    m_params.return_first_solution = false;
    m_params.max_time = 0.0;
    m_params.repair_time = 0.0;

    /// Four Modes:
    ///     Search Until Solution Bounded
    ///     Search Until Solution Unbounded
    ///     Improve Solution Bounded
    ///     Improve Solution Unbounded
}

bool SMHAStar::Init(
    DiscreteSpaceInformation* space,
    Heuristic* anchor,
    Heuristic** heurs,
    int heur_count)
{
    if (!space || !anchor || !heurs || heur_count < 0) {
        return false;
    }
    environment_ = space;
    m_anchor = anchor;
    m_heurs = heurs;
    m_heur_count = heur_count;
    m_open = new OpenList[heur_count + 1];
    return true;
}

SMHAStar::~SMHAStar()
{
    clear();
    if (m_open != NULL) delete[] m_open;
}

int SMHAStar::set_start(int start_state)
{
    m_start_state = get_state(start_state);
    if (!m_start_state) {
        return 0;
    }
    return 1;
}

int SMHAStar::set_goal(int goal_state)
{
    m_goal_state = get_state(goal_state);
    if (!m_goal_state) {
        return 0;
    }
    return 1;
}

int SMHAStar::replan(double allowed_time, std::vector<int>* solution)
{
    int solcost;
    return replan(allowed_time, solution, &solcost);
}

int SMHAStar::replan(double allowed_time, std::vector<int>* solution, int* solcost)
{
    ReplanParams params = m_params;
    params.max_time = allowed_time;
    return replan(solution, params, solcost);
}

int SMHAStar::replan(std::vector<int>* solution, ReplanParams params)
{
    int solcost;
    return replan(solution, params, &solcost);
}

int SMHAStar::replan(std::vector<int>* solution, ReplanParams params, int* solcost)
{
    if (!check_params(params)) { // errors printed within
        return 0;
    }

    m_params = params;

    SMPL_INFO_NAMED(LOG, "Generic Search parameters:");
    SMPL_INFO_NAMED(LOG, "  Initial Epsilon: %0.3f", m_params.initial_eps);
    SMPL_INFO_NAMED(LOG, "  Final Epsilon: %0.3f", m_params.final_eps);
    SMPL_INFO_NAMED(LOG, "  Delta Epsilon: %0.3f", m_params.dec_eps);
    SMPL_INFO_NAMED(LOG, "  Return First Solution: %s", m_params.return_first_solution ? "true" : "false");
    SMPL_INFO_NAMED(LOG, "  Max Time: %0.3f", m_params.max_time);
    SMPL_INFO_NAMED(LOG, "  Repair Time: %0.3f", m_params.repair_time);
    SMPL_INFO_NAMED(LOG, "MHA Search parameters:");
    SMPL_INFO_NAMED(LOG, "  MHA Epsilon: %0.3f", m_initial_eps_mha);
    SMPL_INFO_NAMED(LOG, "  Max Expansions: %d", m_max_expansions);

    environment_->EnsureHeuristicsUpdated(true); // TODO: support backwards search

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    reinit_search();

    m_eps = m_params.initial_eps;
    m_eps_mha = m_initial_eps_mha;
    m_eps_satisfied = (double)INFINITECOST;

    // reset time limits
    m_num_expansions = 0;
    m_elapsed = 0.0;

    double start_time = GetTime();

    ++m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    // insert start state into all heaps with key(start, i) as priority
    for (int i = 0; i < num_heuristics(); ++i) {
        int key = compute_key(m_start_state, i);
        m_start_state->od[i].f = key;
        m_open[i].push(&m_start_state->od[i]);
        SMPL_DEBUG_NAMED(LOG, "Inserted start state %d into search %d with f = %d", m_start_state->state_id, i, key);
    }

    double end_time = GetTime();
    m_elapsed += (end_time - start_time);

    while (!m_open[0].empty() && !time_limit_reached()) {
        start_time = GetTime();

        // special case for mha* without additional heuristics
        if (num_heuristics() == 1) {
            SMPL_WARN_ONCE("Running SMHA* with only the anchor search?");
            if (m_goal_state->g <= get_minf(m_open[0])) {
                m_eps_satisfied = m_eps * m_eps_mha;
                extract_path(solution, solcost);
                return 1;
            } else {
                SMHAState* s = state_from_open_state(m_open[0].min());
                expand(s, 0);
            }
        }

        for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
            if (m_open[0].empty()) {
                break;
            }

            if (!m_open[hidx].empty()) {
                SMPL_DEBUG_NAMED(ELOG, "Compare search %d f_min %d against anchor f_min %d", hidx, get_minf(m_open[hidx]), get_minf(m_open[0]));
            }

            if (!m_open[hidx].empty() && get_minf(m_open[hidx]) <=
                m_eps_mha * get_minf(m_open[0]))
            {
                if (m_goal_state->g <= get_minf(m_open[hidx])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution, solcost);
                    return 1;
                } else {
                    SMHAState* s = state_from_open_state(m_open[hidx].min());
                    expand(s, hidx);
                }
            } else {
                if (m_goal_state->g <= get_minf(m_open[0])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution, solcost);
                    return 1;
                } else {
                    SMHAState* s = state_from_open_state(m_open[0].min());
                    expand(s, 0);
                }
            }
        }
        end_time = GetTime();
        m_elapsed += (end_time - start_time);
    }

    if (m_open[0].empty()) {
        SMPL_DEBUG_NAMED(LOG, "Anchor search exhausted");
    }
    if (time_limit_reached()) {
        SMPL_DEBUG_NAMED(LOG, "Time limit reached");
    }

    return 0;
}

int SMHAStar::force_planning_from_scratch()
{
    return 0;
}

int SMHAStar::force_planning_from_scratch_and_free_memory()
{
    return 0;
}

void SMHAStar::costs_changed(const StateChangeQuery& changes)
{
}

void SMHAStar::costs_changed()
{
}

int SMHAStar::set_search_mode(bool bSearchUntilFirstSolution)
{
    return m_params.return_first_solution = bSearchUntilFirstSolution;
}

void SMHAStar::set_initialsolution_eps(double eps)
{
    m_params.initial_eps = eps;
}

double SMHAStar::get_initial_eps()
{
    return m_params.initial_eps;
}

double SMHAStar::get_solution_eps() const
{
    return m_eps_satisfied;
}

double SMHAStar::get_final_epsilon()
{
    return m_eps_satisfied;
}

double SMHAStar::get_final_eps_planning_time()
{
    return m_elapsed;
}

double SMHAStar::get_initial_eps_planning_time()
{
    return m_elapsed;
}

int SMHAStar::get_n_expands() const
{
    return m_num_expansions;
}

int SMHAStar::get_n_expands_init_solution()
{
    return m_num_expansions;
}

void SMHAStar::get_search_stats(std::vector<PlannerStats>* s)
{
}

void SMHAStar::set_initial_mha_eps(double eps)
{
    m_initial_eps_mha = eps;
}

void SMHAStar::set_final_eps(double eps)
{
    m_params.final_eps = eps;
}

void SMHAStar::set_dec_eps(double eps)
{
    m_params.dec_eps = eps;
}

void SMHAStar::set_max_expansions(int expansion_count)
{
    m_max_expansions = expansion_count;
}

void SMHAStar::set_max_time(double max_time)
{
    m_params.max_time = max_time;
}

double SMHAStar::get_initial_mha_eps() const
{
    return m_initial_eps_mha;
}

double SMHAStar::get_final_eps() const
{
    return m_params.final_eps;
}

double SMHAStar::get_dec_eps() const
{
    return m_params.dec_eps;
}

int SMHAStar::get_max_expansions() const
{
    return m_max_expansions;
}

double SMHAStar::get_max_time() const
{
    return m_params.max_time;
}

bool SMHAStar::check_params(const ReplanParams& params)
{
    if (params.initial_eps < 1.0) {
        SMPL_ERROR("Initial Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.final_eps > params.initial_eps) {
        SMPL_ERROR("Final Epsilon must be less than or equal to initial epsilon");
        return false;
    }

    if (params.dec_eps <= 0.0) {
        SMPL_ERROR("Delta epsilon must be strictly positive");
        return false;
    }

    if (m_initial_eps_mha < 1.0) {
        SMPL_ERROR("MHA Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.return_first_solution &&
        params.max_time <= 0.0 &&
        m_max_expansions <= 0)
    {
        SMPL_ERROR("Max Time or Max Expansions must be positive");
        return false;
    }

    return true;
}

bool SMHAStar::time_limit_reached() const
{
    if (m_params.return_first_solution) {
        return false;
    } else if (m_params.max_time > 0.0 && m_elapsed >= m_params.max_time) {
        return true;
    } else if (m_max_expansions > 0 && m_num_expansions >= m_max_expansions) {
        return true;
    } else {
        return false;
    }
}

SMHAState* SMHAStar::get_state(int state_id)
{
    assert(state_id >= 0 && state_id < environment_->StateID2IndexMapping.size());
    int* idxs = environment_->StateID2IndexMapping[state_id];
    if (idxs[MHAMDP_STATEID2IND] == -1) {
        // overallocate search state for appropriate heuristic information
        size_t state_size =
                sizeof(SMHAState) +
                sizeof(SMHAState::HeapData) * (m_heur_count);
        SMHAState* s = (SMHAState*)malloc(state_size);

        new (s) SMHAState;
        for (int i = 0; i < m_heur_count; ++i) {
            new (&s->od[1 + i]) SMHAState::HeapData;
        }

        size_t mha_state_idx = m_search_states.size();
        init_state(s, mha_state_idx, state_id);

        // map graph state to search state
        idxs[MHAMDP_STATEID2IND] = mha_state_idx;
        m_search_states.push_back(s);

        return s;
    } else {
        int ssidx = idxs[MHAMDP_STATEID2IND];
        return m_search_states[ssidx];
    }
}

void SMHAStar::clear()
{
    clear_open_lists();

    // free states
    for (size_t i = 0; i < m_search_states.size(); ++i) {
        // unmap graph to search state
        int state_id = m_search_states[i]->state_id;
        int* idxs = environment_->StateID2IndexMapping[state_id];
        idxs[MHAMDP_STATEID2IND] = -1;

        // free search state
        free(m_search_states[i]);
    }

    // empty state table
    m_search_states.clear();

    m_start_state = NULL;
    m_goal_state = NULL;
}

void SMHAStar::init_state(SMHAState* state, size_t mha_state_idx, int state_id)
{
    state->call_number = 0; // not initialized for any iteration
    state->state_id = state_id;
    state->closed_in_anc = false;
    state->closed_in_add = false;
    for (int i = 0; i < num_heuristics(); ++i) {
        state->od[i].me = state;
        state->od[i].h = compute_heuristic(state->state_id, i);
    }
}

void SMHAStar::reinit_state(SMHAState* state)
{
    if (state->call_number != m_call_number) {
        state->call_number = m_call_number;
        state->g = INFINITECOST;
        state->bp = NULL;

        state->closed_in_anc = false;
        state->closed_in_add = false;

        for (int i = 0; i < num_heuristics(); ++i) {
            state->od[i].h = compute_heuristic(state->state_id, i);
            state->od[i].f = INFINITECOST;
        }
    }
}

void SMHAStar::reinit_search()
{
    clear_open_lists();
}

void SMHAStar::clear_open_lists()
{
    for (int i = 0; i < num_heuristics(); ++i) {
        m_open[i].clear();
    }
}

int SMHAStar::compute_key(SMHAState* state, int hidx)
{
    return state->g + m_eps * state->od[hidx].h;
}

void SMHAStar::expand(SMHAState* state, int hidx)
{
    SMPL_DEBUG_NAMED(LOG, "Expanding state %d in search %d", state->state_id, hidx);
    SMPL_DEBUG_NAMED(DLOG, "%d, %d", state->state_id, hidx);

    assert(!closed_in_add_search(state) || !closed_in_anc_search(state));

    if (hidx == 0) {
        state->closed_in_anc = true;
    } else {
        state->closed_in_add = true;
    }
    ++m_num_expansions;

    // remove s from all open lists
    for (int i = 0; i < num_heuristics(); ++i) {
        if (m_open[i].contains(&state->od[i])) {
            m_open[i].erase(&state->od[i]);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    environment_->GetSuccs(state->state_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        int cost = costs[sidx];
        SMHAState* succ_state = get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        SMPL_DEBUG_NAMED(LOG, " Successor %d", succ_state->state_id);

        int new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!closed_in_anc_search(succ_state)) {
                int fanchor = compute_key(succ_state, 0);
                succ_state->od[0].f = fanchor;
                insert_or_update(succ_state, 0);
                SMPL_DEBUG_NAMED(LOG, "  Update in search %d with f = %d", 0, fanchor);

                if (!closed_in_add_search(succ_state)) {
                    for (int i = 1; i < num_heuristics(); ++i) {
                        int fn = compute_key(succ_state, i);
                        if (fn <= m_eps_mha * fanchor) {
                            succ_state->od[i].f = fn;
                            insert_or_update(succ_state, i);
                            SMPL_DEBUG_NAMED(LOG, "  Update in search %d with f = %d", i, fn);
                        } else {
                            SMPL_DEBUG_NAMED(LOG, "  Skipping update of in search %d (%0.3f > %0.3f)", i, (double)fn, m_eps_mha * fanchor);
                        }
                    }
                }
            }
        }
    }

    assert(closed_in_any_search(state));
}

SMHAState* SMHAStar::state_from_open_state(SMHAState::HeapData* open_state)
{
    return open_state->me;
}

int SMHAStar::compute_heuristic(int state_id, int hidx)
{
    if (hidx == 0) {
        return m_anchor->GetGoalHeuristic(state_id);
    } else {
        return m_heurs[hidx - 1]->GetGoalHeuristic(state_id);
    }
}

int SMHAStar::get_minf(OpenList& pq) const
{
    return pq.min()->f;
}

void SMHAStar::insert_or_update(SMHAState* state, int hidx)
{
    if (m_open[hidx].contains(&state->od[hidx])) {
        m_open[hidx].update(&state->od[hidx]);
    } else {
        m_open[hidx].push(&state->od[hidx]);
    }
}

void SMHAStar::extract_path(std::vector<int>* solution_path, int* solcost)
{
    SMPL_DEBUG_NAMED(LOG, "Extracting path");
    solution_path->clear();
    *solcost = 0;
    for (SMHAState* state = m_goal_state; state; state = state->bp)
    {
        solution_path->push_back(state->state_id);
        if (state->bp) {
            *solcost += (state->g - state->bp->g);
        }
    }

    // TODO: special cases for backward search
    std::reverse(solution_path->begin(), solution_path->end());
}

bool SMHAStar::closed_in_anc_search(SMHAState* state) const
{
    return state->closed_in_anc;
}

bool SMHAStar::closed_in_add_search(SMHAState* state) const
{
    return state->closed_in_add;
}

bool SMHAStar::closed_in_any_search(SMHAState* state) const
{
    return state->closed_in_anc || state->closed_in_add;
}

} // namespace smpl
