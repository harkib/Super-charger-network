#include <vector>
#include <string>
#include <math.h>
#include <map>
#include <queue>
#include "network.h"
#include <array>
#include<utility>
#include<cstdio>
#include<algorithm>

#include <iostream>


using namespace std;

const double full_charge = 320.00;
const double speed = 105.00;
const double Earth_radius = 6356.752;
//g++ -std=c++11 -O1 main.cpp network.cpp -o candidate_solution
//./candidate_solution Council_Bluffs_IA Cadillac_MI
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



double h(int i, int j, double rate_min){


  //return get_arc_distance(i,j)/speed;

    double h_drive = get_arc_distance(i,j)/speed;
    double h_charge = (get_arc_distance(i,j) - full_charge)/rate_min;
    return h_drive + h_charge;

}

double get_rate_min(int n){
    double rate_min = 999;
    for (int i = 0; i < n; i++){
        if(rate_min > network[i].rate){
            rate_min = network[i].rate;
        }
    }
    return rate_min;
}
double get_rate_max(int n){
  double rate_max = 0;
  for(int i=0;i<n;i++){
    if(rate_max<network[i].rate){
      rate_max = network[i].rate;
    }
  }
}

vector<pair<string,double>> A_star(map< int, map< int, double>> distance_map,int n, std::string start,std::string goal){
    double time_step = .01; //hours // for charging
    double rate_min = get_rate_min(n);
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

    node root;
    root.name = network[start_i].name;
    root.network_index = start_i;
    root.longitude = network[start_i].lon;
    root.latitude = network[start_i].lat;
    root.rate = network[start_i].rate;
    root.car_charge = full_charge;
    root.time = 0;
    root.path.push_back(std::make_pair(root.name,0));
    root.f = h(start_i,goal_i,rate_min);
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
                    x.f = x.time +h(i,goal_i,rate_min);

                    pQueue.push(x);

                    //cout << "added reachable node" << endl;
                }
            } else {
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
                int over_time = (x.car_charge - full_charge)/x.rate;
                x.car_charge = full_charge;
                x.time -= over_time;
                x.path[x.path.size()-1].second -= over_time;

            }
            x.f = x.time + h(x.network_index, goal_i,rate_min);
            pQueue.push(x);

        }

    }

    vector<pair<string,double>> no_solution;
    no_solution.push_back(std::make_pair("No solution",0));
    return  no_solution;
}







std::tuple<double,string,vector<pair<string,double>> >
search_succ(map< int, map< int, double>> distance_map,vector<pair<string,double>> path,
  int n,double rate_min, double g, double threshold,int goal_i,double car_charge)
  {
  node curr;
  pair <string,double>curr_state = path.back();
  curr.name = curr_state.first;
  curr.time = curr_state.second;
  curr.car_charge = car_charge;
  //cout<<"examing "+curr.name<<endl;
  //get index of node
  int curr_i;
  for(int i=0;i<n;i++){
    if (network[i].name == curr.name){
      curr_i = i;
    }
  }
  double f = g + h(curr_i,goal_i,rate_min);
  if (f > threshold){
    std::tuple <double,string,vector<pair<string,double>>> result (f,"ROUTING",path);
    return result;
  }
  //goal test
  if (curr.name == network[goal_i].name){
    std::tuple <double,string,vector<pair<string,double>>> result (f,"FOUND",path);
    cout<<"Goaltest:--reach destination, curr path size: "<<path.size()<<" threshold:"<<std::get<0>(result)<<endl;
    return result;
    //cout<<"cosrr return";
  }
  double min = INFINITY;
  //all reachable succ from node:
  for (int i = 0; i < n; i++){
      if (distance_map[curr_i][i] <= curr.car_charge && curr_i != i){
          //ckeck for looping
          //cout<<"found index "<<i<<" in range"<<endl;
          bool visted = false;
          int path_len = path.size();
          for (int k = 0; k < path_len; k++){
              if(path[k].first == network[i].name){
                  visted = true;
              }
          }

          if(!visted){
              node succ;
              succ.name = network[i].name;
              succ.network_index = i;
              succ.longitude = network[i].lon;
              succ.latitude = network[i].lat;
              succ.rate = network[i].rate;
              succ.car_charge = curr.car_charge - distance_map[curr_i][i];
              cout<<"examing: "<<succ.name<<", energy left: "<<succ.car_charge<<endl;
              cout<<"time :"<<curr.time+distance_map[curr_i][i]/speed<<endl;
              //charging
              double charge_time = 0;
              if (succ.car_charge < full_charge){
                charge_time = (full_charge-succ.car_charge)/network[i].rate;
              }
              //cout<<"examing: "<<succ.name<<"energy left: "<<succ.car_charge<<endl;
              cout<<" cherge time to full charge: "<<charge_time<<endl;

              succ.time = curr.time + distance_map[curr_i][i]/speed + charge_time;
              //succ.path = curr.path;
              succ.f = succ.time +h(i,goal_i,rate_min);
              path.push_back(std::make_pair(succ.name,succ.time));
              //--checking part

              cout<<"push"<<endl;
              //--checked
              std::tuple <double,std::string,vector<pair<string,double>>> t
                        = search_succ(distance_map,path,n,rate_min,succ.time,threshold,goal_i,curr.car_charge);
              if (std::get<1>(t) == "FOUND"){
                double th = std::get<0>(t);
                std::string rs = std::get<1>(t);
                vector<pair<string,double>> p = std::get<2>(t);
                //cout<<"search_succ:--threshold:"<<th<<" result:"<<rs<<" pathsize:"<<p.size()<<endl;
                std::tuple <double,std::string,vector<pair<string,double>>> result (get<0>(t),"FOUND",get<2>(t));
                return result;
              }
              if (std::get<0>(t)<min){
                min = std::get<0>(t);
              }
              cout<<"pop:  -"<<path.back().first<<endl;
              path.pop_back();
              //cout << "added reachable node" << endl;
          }
      }
  }
  std::tuple <double,std::string,vector<pair<string,double>>> result (min,"NOT_FOUND",path);
  return result;
}

vector<pair<string,double>> ID_A_star(map< int, map< int, double>>
  distance_map,int n, std::string start,std::string goal){

    double time_step = 0.01; //hours // for charging
    double rate_min = get_rate_min(n);
    vector<pair<string,double>> path;
    //find index of start and goal
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
    cout<<"---------straight line distance: "<<distance_map[start_i][goal_i] <<"--------------"<<endl;
    //original threshold
    double threshold = h(start_i,goal_i,rate_min);
    node root;
    root.name = network[start_i].name;
    root.network_index = start_i;
    root.longitude = network[start_i].lon;
    root.latitude = network[start_i].lat;
    root.rate = network[start_i].rate;
    root.car_charge = full_charge;
    root.time = 0;
    path.push_back(std::make_pair(root.name,0));
    root.f = h(start_i,goal_i,rate_min);
    cout<<"----------start: "<<root.name<<"initial charge: "<<root.car_charge<<"----------------"<<endl;



    while(1){
      std::tuple <double,std::string,vector<pair<string,double>>> t = search_succ(distance_map,path,n,rate_min,0,threshold,goal_i,root.car_charge);
      if (std::get<1>(t) == "FOUND") {
        double th = std::get<0>(t);
        std::string rs = std::get<1>(t);
        vector<pair<string,double>> p = std::get<2>(t);
        cout<<"ID_A_star:--threshold:"<<th<<" result:"<<rs<<" pathsize:"<<p.size()<<endl;
        return std::get<2>(t);
      }
      if (std::get<0>(t) == INFINITY){
        vector<pair<string,double>> no_solution;
        no_solution.push_back(std::make_pair("No solution",0));
        return  no_solution;
      }
      threshold = std::get<0>(t);

    }


}


/*struct look_aheadSave
{
    int index;
    bool usedvisted;
};
*/


vector<pair<string,double>> AL_star(map< int, map< int, double>> distance_map,int n, std::string start,std::string goal){
    cout<<"-----------------------"<<"Start Runing AL_star"<<"---------------------"<<endl;
    
    cout<<"Trip from "<<start<<" to "<<goal<<endl;
    cout<<"initial charge of car is "<< full_charge<<endl;
    double time_step = .01; //hours // for charging
    double rate_min = get_rate_min(n);
    priority_queue<node, vector<node>, compare_node_f> pQueue;
    priority_queue<node, vector<node>, compare_node_f> QlookaheadSecond;//a new queue for the second step lookahead while check.

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
    cout<<"---------straight line distance: "<<distance_map[start_i][goal_i] <<"--------------"<<endl;

    node root;
    root.name = network[start_i].name;
    root.network_index = start_i;
    root.longitude = network[start_i].lon;
    root.latitude = network[start_i].lat;
    root.rate = network[start_i].rate;
    root.car_charge = full_charge;
    root.time = 0;
    root.path.push_back(std::make_pair(root.name,0));
    root.f = h(start_i,goal_i,rate_min);
    pQueue.push(root);

    while (!pQueue.empty()){
        node curr = pQueue.top();
        pQueue.pop();
        if (curr.name == goal ){

            return curr.path;
        }


        //add reachable nodes
        for (int i = 0; i < n; i++){

            //reachable nodes
            if (distance_map[curr.network_index][i] <= curr.car_charge && curr.network_index != i){
                //ckeck for looping
                bool trackCheck = true;
                int path_len = curr.path.size();
                for (int k = 0; k < path_len; k++){
                    if(curr.path[k].first == network[i].name){
                        trackCheck = false;
                    }
                }
 /*       for(int i=0; i<n;i++)
        {

            if (distance_map[curr.network_index][i] <= curr.car_charge && curr.network_index != i)
            {
                for (int k = 0; k < path_len; k++)
                {
                    if(curr.path[k].first == network[i].name)
                    {
                        tempsave.push(i);
                    }
                }
            }
        }
        int lookaheadcpr[tempsave.size()];
        
        int lpsize=tempsave.size();
        for(int u=0;u<tempsave.size();u++)
        {
            int findIdex=tempsave.top();
            tempsave.pop();
            lookaheadcpr[u]=findIdex;

        }
        
        for(int j=0;j<lpsize;j++)
        {
            for(int i=0; i<n;i++)
            {

                if (distance_map[lookaheadcpr[j]][i] <= curr.car_charge && curr.network_index != i)
                {
                    for (int k = 0; k < path_len; k++)
                    {
                        if(curr.path[k].first == network[i].name)
                        {
                            tempsave2.push(i);
                        }
                    }
                }
            }
        }
        pair<int,double> lookaheadcpr1[tempsave2.size()];
        for(int j=0;j<lpsize;j++)
        {
            for(int i=0; i<n;i++)
            {

                if (distance_map[lookaheadcpr[j]][i] <= curr.car_charge && curr.network_index != i)
                {
                    for (int k = 0; k < path_len; k++)
                    {
                        if(curr.path[k].first == network[i].name)
                        {
                            lookaheadcpr1[j]=make_pair(lookaheadcpr[j],i);
                        }
                    }
                }
            }
        }
        double savingtime[];
        for(int i=0;i<tempsave2.size();i++)
        {
            savingtime[i]=curr.time + distance_map[curr.network_index][lookaheadcpr1[i].first]/speed+distance_map[lookaheadcpr1[i].first][lookaheadcpr1[i].second];
        }
        int min_num=0;
        int cprmin=savingtime[0];
        for(int i=0;i<size(savingtime)-1;i++)
        {
            if(cprmin>savingtime[i+1])
            {
                cprmin=savingtime[i+1];
                min_num=i+1;
            }

        }
        */
                if(trackCheck){
                    node lookaheadfirststep;
                    lookaheadfirststep.name = network[i].name;
                    lookaheadfirststep.network_index = i;
                    lookaheadfirststep.longitude = network[i].lon;
                    lookaheadfirststep.latitude = network[i].lat;
                    lookaheadfirststep.rate = network[i].rate;
                    lookaheadfirststep.car_charge = curr.car_charge - distance_map[curr.network_index][i];
                    lookaheadfirststep.time = curr.time + distance_map[curr.network_index][i]/speed;
                    lookaheadfirststep.path = curr.path;
                    lookaheadfirststep.path.push_back(std::make_pair(lookaheadfirststep.name,0));
                    lookaheadfirststep.f = lookaheadfirststep.time +h(i,goal_i,rate_min);
                    cout<<"station: "<<lookaheadfirststep.name<<"    charging left: "<<lookaheadfirststep.car_charge<<endl;
                    cout<<"time :"<<lookaheadfirststep.time<<endl;

                    pQueue.push(lookaheadfirststep);

                    
                }
            } else {
                //tracked or dissatisfy
            }
        }

        //add charging nodes
        if (curr.car_charge < full_charge) {
            node x = curr;
            x.time += time_step;
            x.car_charge += time_step*x.rate;
            x.path[x.path.size()-1].second += time_step;
            if (x.car_charge > full_charge){
                int over_time = (x.car_charge - full_charge)/x.rate;
                x.car_charge = full_charge;
                x.time -= over_time;
                x.path[x.path.size()-1].second -= over_time;

            }
            x.f = x.time + h(x.network_index, goal_i,rate_min);
            pQueue.push(x);

        }
        //lookahead one more step 
        while (!QlookaheadSecond.empty()){
          curr = QlookaheadSecond.top();
          QlookaheadSecond.pop();

          if (curr.name == goal ){

            return curr.path;
          }


        //add reachable nodes
          for (int i = 0; i < n; i++){

            //reachable nodes
              if (distance_map[curr.network_index][i] <= curr.car_charge && curr.network_index != i){
                //ckeck for looping
                  bool trackChecksecond = true;
                  int path_len = curr.path.size();
                  for (int k = 0; k < path_len; k++){
                      if(curr.path[k].first == network[i].name){
                          trackChecksecond = false;
                      }
                  }

                  if(trackChecksecond){
                    node lookaheadsecondstep;
                    lookaheadsecondstep.name = network[i].name;
                    lookaheadsecondstep.network_index = i;
                    lookaheadsecondstep.longitude = network[i].lon;
                    lookaheadsecondstep.latitude = network[i].lat;
                    lookaheadsecondstep.rate = network[i].rate;
                    lookaheadsecondstep.car_charge = curr.car_charge - distance_map[curr.network_index][i];
                    lookaheadsecondstep.time = curr.time + distance_map[curr.network_index][i]/speed;
                    lookaheadsecondstep.path = curr.path;
                    lookaheadsecondstep.path.push_back(std::make_pair(lookaheadsecondstep.name,0));
                    lookaheadsecondstep.f = lookaheadsecondstep.time +h(i,goal_i,rate_min);
                    cout<<"station: "<<lookaheadsecondstep.name<<"    charging left: "<<lookaheadsecondstep.car_charge<<endl;
                    cout<<"time :"<<lookaheadsecondstep.time<<endl;

                    pQueue.push(lookaheadsecondstep);

                    //cout << "added reachable node" << endl;
                  }
            } 
            else 
            {
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
                int over_time = (x.car_charge - full_charge)/x.rate;
                x.car_charge = full_charge;
                x.time -= over_time;
                x.path[x.path.size()-1].second -= over_time;

            }
            x.f = x.time + h(x.network_index, goal_i,rate_min);
            pQueue.push(x);

        }

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
    //cout << "Running A-star..." << endl;
    //vector<pair<string,double>> path1 = A_star(distance_map,n, start_charger_name, end_charger_name);
    //print_path(path1);
    cout << "Running AL_star..." << endl;
    vector<pair<string,double>> path2 = AL_star(distance_map,n, start_charger_name, end_charger_name);
    //cout<<path2.size();

    //print path
    print_path(path2);

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
