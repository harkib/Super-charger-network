#include "network.h"

using namespace std;

struct node {
    std::string name;
    int network_index;
    double latitude;
    double longitude;
    double rate;
    double car_charge;
    double time; //g
    //path_to, another struct ? 
    double f; //h is time to drive arc to goal 
    
};

/*
arc_distnace(network, i, j){
    double distance = pow(pow(network[i].lat - network[j].lat, 2) + pow(network[i].lon - network[j].lon, 2),.5)
    double theta = arcsin (distance/(2*Earth_radius))

    retrun theta*Earth_radius

}
*/

/*gen_distance_map(network, full_charge){
    map< int, map< int, double>>;
    //map< network_index1, map< network_index2, double> > distance;
    for i,j to n
        if arc_distance <= full_charge
            distance_map[i][j] = arc_distnace

    return distance_map
}*/



//h(node, goal_node) = arc_distance/speed = time 


/*A_star(distance_map, network, start, goal){
    time_step = .1 hours // for charging 

    root = node of start;
    pQueue.push(root)

    while pQueue.not_empty{
        curr = pQueue.pop

        //do we need upper bound on time cost?

        //Check goal condition 
        if (curr.name == goal ){
            reurtn curr.path_to
        }

        
        //add reachable nodes
        for i to n
            if distance_map[curr.network_index][i] <= curr.car_charge
                node x;
                x.name = netowrk[i].name
                x.network_index = i;
                x.longitude = netowrk[i].longitude;
                x.rate = netowrk[i].rate;
                x.car_charge = curr.car_charge - distance_map[curr.network_index][i];
                x.time = curr.time + distance_map[curr.network_index][i]/speed;
                //path_to, another struct ? 
                x.f = x.time +h(x,goal)
                
                pQueue.push(x)

        //add charging nodes
        if curr.car_charge != full_charge {
            node x = curr;
            x.time = x.time + time_step;
            x.car_charge = x.car_charge + time_step*x.rate;
        s}

    }

    // return no solutions 
}*/

int main(int argc, char** argv)
{
    //get user input for charger names 

    std::string start_charger_name;
    std::string end_charger_name;

    //maybe gobal 
    double full_charge = 320.00;
    double speed = 105.00;
    double Earth_radius = 6356.752;

    
    //A_star(distance_map, network, start, goal)

    return 0;
}
