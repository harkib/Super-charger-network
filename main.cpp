#include <vector>
#include <string>
#include <math.h>
#include <map>
#include <queue> 
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
    double rate; //km/h
    double car_charge; //km
    double time; //g //hours
    vector<pair<string,double>> path;
    double f; //h is time to drive arc to goal 
    
};

struct compare_node_f { 
    bool operator()(node const& n1, node const& n2) 
    { 

        return n1.f > n2.f; 
    } 
}; 

double get_arc_distance(int i, int j){
    double NS_straight = 2*sin((3.14/180)*abs(network[i].lon - network[j].lon)/2)*Earth_radius;
    double EW_straight = 2*sin((3.14/180)*abs(network[i].lat - network[j].lat)/2)*Earth_radius;
    double distance_straight = pow(pow(EW_straight, 2) + pow(NS_straight, 2),.5);
    double theta = 2*asin ((distance_straight/2)/Earth_radius);

    return theta*Earth_radius;

}

void print_path(vector<pair<string,double>> path ){
    int n = path.size();
    cout << path[0].first<< ", ";
    for (int i = 1; i < n - 1; i++){
        cout << path[i].first << ", " << path[i].second << ", ";
    }
    if (n > 1){
        cout << path[n-1].first<< endl;
    } else {
        cout << endl;
    }
}

map< int, map< int, double>> gen_distance_map(int n){
    //map< network_index1, map< network_index2, double> > distance;
    map< int, map< int, double>> distance_map;
    
    double arc_distance;  
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            if (i!=j){
                arc_distance = get_arc_distance(i, j);
                distance_map[i][j] = arc_distance;

                }
            }
    }
    return distance_map;
}

 

double h(int i, int j, double rate_max, double curr_charge){
 
    //minimum drive time
    double h_drive = get_arc_distance(i,j)/speed; 

    //minimum charge time 
    //double possible_charging_at_curr = full_charge - curr_charge;
    //double h_charge = (get_arc_distance(i,j) - full_charge)/rate_min + possible_charging_at_curr/network[i].rate;
    double h_charge = (get_arc_distance(i,j) - curr_charge)/rate_max;
    if (h_charge < 0){
        h_charge = 0;
    }
    return h_drive + h_charge;
     
} 

double get_rate_max(int n){
    double rate_max = 0;
    for (int i = 0; i < n; i++){
        if(rate_max < network[i].rate){
            rate_max = network[i].rate;
        }
    }
    return rate_max;
}

vector<pair<string,double>> A_star(map< int, map< int, double>> distance_map,int n, std::string start,std::string goal){
    double time_step = .1; //hours // for charging 
    double rate_max = get_rate_max(n);
    cout << "Running A-star... with time_step = "<< time_step <<", rate_max = "<<rate_max << endl;
    
    priority_queue<node, vector<node>, compare_node_f> pQueue;

    // look up index of start node name
    int start_i = -1; //start index
    int goal_i = -1;
    for (int j = 0; j < n; j++){
        if (network[j].name == start){
            start_i = j;
        }
        if (network[j].name == goal){
            goal_i = j;
        }
    }

    //create root node
    node root;
    root.name = network[start_i].name;
    root.network_index = start_i;
    root.longitude = network[start_i].lon;
    root.latitude = network[start_i].lat;
    root.rate = network[start_i].rate;
    root.car_charge = full_charge;
    root.time = 0;
    root.path.push_back(std::make_pair(root.name,0));
    root.f = h(start_i,goal_i,rate_max, root.car_charge);
    pQueue.push(root);

    while (!pQueue.empty()){
        node curr = pQueue.top();
        pQueue.pop();
        //cout << curr.name << endl;

        //print_path(curr.path);
        // cout << curr.f << endl;

        

        //do we need upper bound on time cost?

        //Check goal condition 
        if (curr.name == goal ){

            return curr.path;
        }

        
        //add reachable nodes
        for (int i = 0; i < n; i++){

            //reachable nodes 
            if (distance_map[curr.network_index][i] <= curr.car_charge && curr.network_index != i){

                //ckeck for looping 
                bool visted = false; 
                int path_len = curr.path.size();
                for (int k = 0; k < path_len; k++){
                    if(curr.path[k].first == network[i].name){
                        visted = true;
                    }
                }

                //if not visited
                if(!visted){
                    node x;
                    x.name = network[i].name;
                    x.network_index = i;
                    x.longitude = network[i].lon;
                    x.latitude = network[i].lat;
                    x.rate = network[i].rate;
                    x.car_charge = curr.car_charge - distance_map[curr.network_index][i];
                    x.time = curr.time + distance_map[curr.network_index][i]/speed;
                    x.path = curr.path;
                    x.path.push_back(std::make_pair(x.name,0));
                    x.f = x.time +h(i,goal_i,rate_max,x.car_charge);
                    //x.f = x.time +h(i,goal_i,rate_min,full_charge);
                    pQueue.push(x);

                    //cout << "added reachable node" << endl;
                }          
            } 
            //rechable with charging  
            // else if(distance_map[curr.network_index][i] <= full_charge && curr.network_index != i){

            //     //ckeck for looping 
            //     bool visted = false; 
            //     int path_len = curr.path.size();
            //     for (int k = 0; k < path_len; k++){
            //         if(curr.path[k].first == network[i].name){
            //             visted = true;
            //         }
            //     }

            //     //if not visited
            //     //stays at same charger but charges minimum to get to charger i 
            //     if(!visted){
            //         double charge_needed = distance_map[curr.network_index][i] - curr.car_charge;
            //         double time_needed = charge_needed/curr.rate;

            //         node x = curr;
            //         x.car_charge += charge_needed;
            //         x.time +=  time_needed;
            //         x.path[x.path.size()-1].second += time_needed;
            //         x.f = x.time +h(x.network_index,goal_i,rate_min,x.car_charge);
                    
            //         pQueue.push(x);
            //     }

            // }
            else {
                //cout << "not reachble" << endl;
            }
        }

        //add charging nodes
        if (curr.car_charge < full_charge) { 
            node x = curr;
            x.time += time_step;
            x.car_charge += time_step*x.rate;  
            x.path[x.path.size()-1].second += time_step;
            if (x.car_charge > full_charge){
                float over_time = (x.car_charge - full_charge)/x.rate;
                x.car_charge = full_charge;
                x.time -= over_time;
                x.path[x.path.size()-1].second -= over_time;

            }
            x.f = x.time + h(x.network_index, goal_i,rate_max,x.car_charge);
            //x.f = x.time + h(x.network_index, goal_i,rate_min,full_charge);
            pQueue.push(x);

        }

    }

    vector<pair<string,double>> no_solution;
    no_solution.push_back(std::make_pair("No solution",0));
    return  no_solution;
}

bool valid_user_input(std::string location, int n){
    bool found = false;
    for (int i = 0; i < n; i++){
        if (network[i].name == location){
            found = true;
        }
    }
    return found;
}

void check_path(vector<pair<string,double>> path, map< int, map< int, double>> distance_map, int n){
    vector<pair<int,double>> path_indexed;
    double remaining_charge = full_charge;
    double drive_dis = 0;

    //convert names to netwrok indexs
    for (int k = 0; k < path.size(); k++){
        for (int j = 0; j < n; j++){
            if (network[j].name == path[k].first){
                path_indexed.push_back(make_pair(j,path[k].second));
            }
        }
    }

    //compute
    int curr, next;
    for (int k = 0; k < path_indexed.size() -1; k++){

        //compute drive distance
        curr = path_indexed[k].first;
        next = path_indexed[k+1].first;
        drive_dis += distance_map[curr][next];

        //compute remaining charge
        remaining_charge -= distance_map[curr][next];
        cout <<"Drove "<<distance_map[curr][next] << " km to "<< path[k+1].first <<", charge (km) reamining = "<<remaining_charge << endl;
        

        if(remaining_charge < 0){
            cout <<"Charge below Zero!" << endl;
        }
        remaining_charge += network[next].rate*path_indexed[k+1].second;
        cout <<network[next].name <<" Charger speed: "<< network[next].rate << ",Charged to (km):" << remaining_charge <<endl;
    }


    cout << "Remaining charge (km) (should be close to 0): " << remaining_charge << endl; 
    cout << "Drive distance (compare with google): " << drive_dis << endl;

}


int main(int argc, char** argv)
{   
    //should exapnd 
    const int nMax = 303;

    //vary n ...
    int n = 303;


    //check input
    if (argc != 3){
        cout << "Invalid syntax" << endl;
        return 1;
    }

    std::string start_charger_name = argv[1];
    std::string end_charger_name = argv[2];

     if(!valid_user_input(start_charger_name, n)){
        cout <<"start_charger_name invalid" << endl;
        return 1;
    }else if(!valid_user_input(end_charger_name, n)){
        cout <<"end_charger_name invalid" << endl;
        return 1;
    }

    //create distance map 
    map< int, map< int, double>> distance_map = gen_distance_map(n);

    //cout << network[0].name << ", " << network[1].name << ": " << distance_map[0][1] << endl;
    vector<pair<string,double>> path = A_star(distance_map,n, start_charger_name, end_charger_name);

    //print path
    print_path(path);
    check_path(path, distance_map,n);

    //debug
    // look up index of start node name
    // int start_i = -1; //start index
    // int goal_i = -1;
    // for (int j = 0; j < n; j++){
    //     if (network[j].name == start_charger_name){
    //         start_i = j;
    //     }
    //     if (network[j].name == end_charger_name){
    //         goal_i = j;
    //     }
    // }
    //cout << "h: " <<h(start_i,goal_i) << endl;
    //cout <<"distance_map: " << distance_map[start_i][goal_i] <<endl;

    return 0;
}
