#include <vector>
#include <string>
#include <math.h>
#include <map>
#include "network.cpp"

using namespace std;

const double full_charge = 320.00;
const double speed = 105.00;
const double Earth_radius = 6356.752;

struct node {
    std::string name;
    int network_index;
    double latitude;
    double longitude;
    double rate;
    double car_charge;
    double time; //g
    vector<pair<string,double>> path;
    double f; //h is time to drive arc to goal 
    
};


double get_arc_distance(int i, int j){
    double NS_straight = 2*sin((3.14/180)*abs(network[i].lon - network[j].lon)/2)*Earth_radius;
    double EW_straight = 2*sin((3.14/180)*abs(network[i].lat - network[j].lat)/2)*Earth_radius;
    double distance_straight = pow(pow(EW_straight, 2) + pow(NS_straight, 2),.5);
    double theta = 2*asin ((distance_straight/2)/Earth_radius);

    return theta*Earth_radius;

}


map< int, map< int, double>> gen_distance_map(int n){
    //map< network_index1, map< network_index2, double> > distance;
    map< int, map< int, double>> distance_map;
    
    double arc_distance;
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            if (i!=j){
                arc_distance = get_arc_distance(i, j);
                //if (arc_distance <= full_charge)
                    distance_map[i][j] = arc_distance;
                //}
                }
            }
    }
    return distance_map;
}



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
                // revenue.push_back(std::make_pair("string",map[i].second));

        //add charging nodes
        if curr.car_charge != full_charge {
            node x = curr;
            x.time = x.time + time_step;
            x.car_charge = x.car_charge + time_step*x.rate;
        s}

    }

    // return no solutions 
}*/

bool valid_user_input(std::string location, int n){
    bool found = false;
    for (int i = 0; i < n; i++){
        if (network[i].name == location){
            found = true;
        }
    }
    return found;
}

int main(int argc, char** argv)
{   

    //should exapnd 
    const int nMax = 303;

    //vary n ...
    int n = 303;

    //get user input for charger names 
    std::string start_charger_name;
    std::string end_charger_name;
    
    // while(!valid_user_input(start_charger_name, n)){
    //     cout << "Enter start_charger_name: ";
    //     cin >> start_charger_name;
    // }
    // while(!valid_user_input(end_charger_name, n)){
    //     cout << "Enter end_charger_name: ";
    //     cin >> end_charger_name;
    // }

    //create distance map 
    map< int, map< int, double>> distance_map = gen_distance_map(n);

    cout << network[0].name << ", " << network[1].name << ": " << distance_map[0][1] << endl;

   // vector<pair<string,double>> path = A_star(distance_map,n, start, goal);

    //print path

    return 0;
}
