#include <vector>
#include <string>
#include <math.h>
#include <map>
#include <queue>
#include "network.h"
#include <array>
#include <stack>
#include<utility>
#include<cstdio>
#include<algorithm>

#include <iostream>
#include<ctime>

using namespace std;

const double full_charge = 320.00;
const double speed = 105.00;
const double Earth_radius = 6356.752;
double time_step = 0.01;

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



double h(int i, int j, double rate_max,double curr_charge){


  //return get_arc_distance(i,j)/speed;

    double h_drive = get_arc_distance(i,j)/speed;
    double h_charge = 0;

    if (curr_charge < get_arc_distance(i,j)){
      double h_charge = (get_arc_distance(i,j) - curr_charge)/rate_max;
    }

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
    double rate_max = get_rate_max(n);
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
    root.f = h(start_i,goal_i,rate_max,root.car_charge);
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
                    x.f = x.time +h(i,goal_i,rate_max,x.car_charge);

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
            x.f = x.time + h(x.network_index, goal_i,rate_max,x.car_charge);
            pQueue.push(x);

        }

    }

    vector<pair<string,double>> no_solution;
    no_solution.push_back(std::make_pair("No solution",0));
    return  no_solution;
}





//stack<node> path;
vector<pair<string,double>> path;
int num_expandable = 1;
//priority_queue<node, vector<node>, compare_node_f> pQueue2;


std::tuple<double,string,vector<pair<string,double>> >
search_succ(vector<pair<string,double>> path,double g,map< int, map< int, double>> distance_map,
  int n,double rate_max, double threshold,int start_i,int goal_i,double car_charge,node curr)
  {
    //vector<string> i;
    //print_path(path);

    //curr = pQueue2.top();
    //pQueue2.pop();
  //node curr;
  //curr.name = path.back().first;
  //curr.car_charge = car_charge;
  //i = 0;
  //i.push_back(curr.name);
  //cout<< "car-charge "<<curr.car_charge<<endl;
  //pair <string,double>curr_state = path.back();

  //curr.car_charge = car_charge;
  //cout<<"threshold: "<<threshold<<endl;
  //get index of node

  curr.f = g + h(curr.network_index,goal_i,rate_max,curr.car_charge);
  //cout<<"f: "<<f<<endl;



  //if (curr.f > threshold){
    //cout<<"return f"<<"curr f "<<curr.f<<" th "<<threshold<<endl;
  //  std::tuple <double,string,vector<pair<string,double>>> result (curr.f,"ROUTING",path);
  //  return result;
 // }
  //if (curr.name == network[goal_i].name){
  //  std::tuple <double,string,vector<pair<string,double>>> result (curr.f,"FOUND",path);
    //cout<<"Goaltest:--reach destination, curr path size: "<<path.size()<<" threshold:"<<std::get<0>(result)<<endl;
  //  return result;
    //cout<<"cosrr return";
  //}
  //goal test

  double min = 9999;
  //add charge node
  //cout<<"curr charge:"<<curr.car_charge<<endl;
  if (curr.f > threshold){
    //cout<<"return f"<<endl;
    std::tuple <double,string,vector<pair<string,double>>> result (curr.f+0.1,"ROUTING",path);
    return result;
  }
  if (curr.name == network[goal_i].name){
    std::tuple <double,string,vector<pair<string,double>>> result (curr.f+0.1,"FOUND",path);
    //cout<<"Goaltest:--reach destination, curr path size: "<<path.size()<<" threshold:"<<std::get<0>(result)<<endl;
    return result;
    //cout<<"cosrr return";
  }

  //all reachable succ from node:
  bool expandable = false;
  //cout<<"                            expanding "<<curr.name<<endl;
  for (int i=0; i < n; i++){
   //cout<<i<<endl;
   //cout<<"---------------------------------------------------------------------curr charge:"<<curr.car_charge<<endl;
      if (distance_map[curr.network_index][i] <= full_charge && curr.network_index != i ){
          expandable = true;
          //ckeck for looping
          //cout<<"found index "<<i<<" in range"<<endl;
          //cout<<"dist to next node: "<<distance_map[curr.network_index][i]<<"  curr charge"<<curr.car_charge<<endl;
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
              succ.car_charge = curr.car_charge - distance_map[curr.network_index][i];
              if (succ.car_charge<0){
                succ.car_charge = 0;
              }
              //cout<<"succ carcharge: "<<succ.car_charge<<"  succ name "<<succ.name<<endl;

              //charging
              double charge_time = 0;

              succ.time = curr.time + distance_map[curr.network_index][i]/speed;
              //succ.path = curr.path;
              succ.f = succ.time +h(i,goal_i,rate_max,succ.car_charge);
              if (h(i,goal_i,rate_max,succ.car_charge) > h(curr.network_index,goal_i,rate_max,curr.car_charge) ){
                //cout<<"skip "<<threshold<<endl;
                continue;
              }
              if(distance_map[curr.network_index][i] > curr.car_charge){
                //charge
                double charge_needed = distance_map[curr.network_index][i] - curr.car_charge;
                curr.car_charge = distance_map[curr.network_index][i];
                curr.time += charge_needed/curr.rate;
                cout<<curr.name<<" "<<succ.name<<" "<<succ.f<<endl;
                path.back().second += charge_needed/curr.rate;
                //pQueue2.push(curr);
                curr.f = curr.time + h(curr.network_index,goal_i,rate_max,curr.car_charge);


              }

              path.push_back(make_pair(succ.name,charge_time));
              //pQueue2.push(succ);
              num_expandable += 1;
              //i.push_back(succ.name);
              //s.push(succ);
              //cout<<path.size()<<endl;
              //cout<<"----------------------------num i  "<<i.size()<<endl;
              //--checking part

              //cout<<"push"<<" car charge: "<<succ.car_charge<<endl;
              //--checked
              //cout<<"serch"<<endl;
              std::tuple <double,std::string,vector<pair<string,double>>> t
                        = search_succ(path,succ.time,distance_map,n,rate_max,threshold,start_i,goal_i,succ.car_charge,succ);
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
              //cout<<"pop:  -"<<path.back().first<<endl;
              path.pop_back();
              //pQueue2.pop();
              //s.pop();
              //cout << "added reachable node" << endl;
          }
      }
  }
  //cout<<"                                   end of expansion for "<<curr.name<<endl;



  if(min == 9999){
    //cout<<" end "<<endl;
  }

  std::tuple <double,std::string,vector<pair<string,double>>> result (min,"NOT_FOUND",path);
  return result;
}

vector<pair<string,double>> ID_A_star(map< int, map< int, double>>
  distance_map,int n, std::string start,std::string goal){

    double time_step = 0.01; //hours // for charging
    double rate_max = get_rate_max(n);




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

    node root;
    root.name = network[start_i].name;
    root.network_index = start_i;
    root.longitude = network[start_i].lon;
    root.latitude = network[start_i].lat;
    root.rate = network[start_i].rate;
    root.car_charge = full_charge;
    root.time = 0;
    double threshold = h(start_i,goal_i,rate_max,root.car_charge);
    root.f = h(start_i,goal_i,rate_max,root.car_charge);
    path.push_back(std::make_pair(root.name,0));
    //pQueue2.push(root);

    num_expandable+=1;
    //i.push_back(root.name);
    //s.push(root);

    cout<<"----------start: "<<root.name<<"initial charge: "<<root.car_charge<<"----------------"<<endl;



    while(1){
      num_expandable = 0;
      //priority_queue<node, vector<node>, compare_node_f> pQueue2;

      std::tuple <double,std::string,vector<pair<string,double>>> t = search_succ(path,0,distance_map,n,rate_max,threshold,start_i,goal_i,root.car_charge,root);
      if (std::get<1>(t) == "FOUND") {
        double th = std::get<0>(t);
        std::string rs = std::get<1>(t);
        vector<pair<string,double>> p = std::get<2>(t);
        cout<<"ID_A_star:--threshold:"<<th<<" result:"<<rs<<" pathsize:"<<p.size()<<endl;
        return std::get<2>(t);
      }
      if (std::get<0>(t) == 9999){
        vector<pair<string,double>> no_solution;
        no_solution.push_back(std::make_pair("No solution",0));
        return  no_solution;
      }
      threshold = std::get<0>(t);
      cout<<"---------------------outer loop  "<<"threshold"<<threshold<<" num_expandable: "<<num_expandable<<endl<<endl;
      //cout<<"outer loop, threshold: "<< threshold<<endl;

    }


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
    clock_t startTime,endTime;
	startTime=clock();
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
    cout << "Running IDA*..." << endl;
    vector<pair<string,double>> path2 = ID_A_star(distance_map,n, start_charger_name, end_charger_name);
    cout<<"# nodes expanded: "<<num_expandable<<endl;
    //cout<<path2.size();

    //print path
    print_path(path2);


    endTime=clock();
    cout<<"The run time is:  "<<(double)(endTime-startTime)/CLOCKS_PER_SEC<<endl;


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
