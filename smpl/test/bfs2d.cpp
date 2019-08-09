#include <iostream>
#include <smpl/bfs/bfs2d.h>


void printMessage(std::string msg){
    std::cout<<msg<<"\n";
    std::string separator(msg.size(), '=');
    std::cout<<separator<<"\n";
}

int main(){
    printMessage("Testing SMPL BFS2D");
    smpl::BFS_2D bfs( 10, 10 );
    printMessage("BFS2D successfully constructed");
    bfs.printGrid();

    std::cout<<"\n";
    printMessage("Adding obstacles");
    bfs.setWall(4, 4);
    bfs.setWall(5, 4);
    bfs.setWall(4, 5);
    bfs.setWall(5, 5);
    bfs.printGrid();

    bfs.run(1, 1);
    int x, y;
    x = 1, y = 1;
    int dist = bfs.getDistance(x, y);
    std::cout<<"Distance of ("<<x<<", "<<y<<") to start: "<<dist<<"\n";
    x = 8, y = 1;
    dist = bfs.getDistance(x, y);
    std::cout<<"Distance of ("<<x<<", "<<y<<") to start: "<<dist<<"\n";
    x = 1, y = 8;
    dist = bfs.getDistance(x, y);
    std::cout<<"Distance of ("<<x<<", "<<y<<") to start: "<<dist<<"\n";
    x = 8, y = 8;
    dist = bfs.getDistance(x, y);
    std::cout<<"Distance of ("<<x<<", "<<y<<") to start: "<<dist<<"\n";
}
