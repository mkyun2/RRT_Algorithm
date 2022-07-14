#include <iostream>
#include <string>
#include <random>
#include <time.h>
#include <vector>
#include <algorithm>
#include <fstream>
using namespace std;

#define thrhold 1

const int width = 300;
const int height = 300;
const double jump = 5;//(width/100.0 * height/100.0)/1.5;+
const int start_x = 102;
const int start_y = 102;
const int des_x = 10;
const int des_y = 22;

struct Node{
    
    int x = 0;
    int y = 0;
    Node *parent;   
};
vector<Node*> nodeList;

vector<pair<int, int>> objectList;
double randSample(int size)
{
    std::random_device random_device;
    std::mt19937 engine(random_device());
    std::uniform_int_distribution<int> distribution(0.0, size);

    return distribution(engine);

}
Node* Sampling(int rwidth, int rheight){
    Node *sample = new Node;// Random Sample
    //Node *tmpNode;
    double dist, angle;
    double mindist=1000.0;
    int i = 0;
    int index;
    sample->x = rwidth;//randSample(width);
    sample->y = rheight;//randSample(height);
    for(auto a: nodeList)
    {
        dist = sqrt(pow(a->x-sample->x,2)+pow(a->y-sample->y,2)); // Calc dist to find minimum Qnear to Qrand 
        
        if(dist < mindist)
        {
            mindist = dist;
            index = i;
            //cout <<"index:" << index <<":" << a->x << ", " << a->y << "dist: "<< dist <<  endl;
            //cout <<"sample:"  << sample->x << ", " << sample->y  <<  endl;
            angle = atan2f(sample->y-a->y,sample->x-a->x); //x cosangle y sin angle
        }
        i++;
    }
    free(sample);

    Node *nNode = new Node;// New node between Qnear and Qrand
    nNode -> x = nodeList[index]->x + jump*cosf(angle);
    nNode -> y = nodeList[index]->y + jump*sinf(angle);
    nNode -> parent = nodeList[index];

    return nNode;

}

bool collision(Node *sample)
{
    Node *parent;
    if (sample->x >width || sample->x < 0)
    {
        free(sample);
        return false;
    } 
    else if(sample->y > height || sample->y < 0)
    {
        free(sample);
        return false;
    }
    else
    {
        for(auto a : objectList)
        {
            bool collision = false;
            double tmpDist = sqrt(pow((a.first-sample->x),2)+pow((a.second-sample->y),2));
            if(tmpDist < 10)
                return false;
            
        }
        nodeList.push_back(sample);
        return true;
    }
    
}
void conNode(Node *nNode)
{
    double mindist = 1000.0;
    double tmpdist=1000.0;
    Node *tmpNode = nNode;
    
    for(auto a : nodeList)
    {

        tmpdist = sqrt(pow((a->x-tmpNode->x),2)+pow((a->y-tmpNode->y),2));
        //cout << "dist.." << tmpdist << ","<< mindist << endl;
        //cout << "dist2.." << tmpdist << ","<< mindist << endl;
         if(tmpdist < thrhold)
             return;
        if(tmpdist < mindist)
            {
                tmpNode->parent = nullptr;
                
                //cout << "dist2.." << tmpdist << ","<< mindist << endl;
                mindist = tmpdist;
                tmpNode->parent = a;
                //cout << "nNode node.." << nNode->x << ", " << nNode->y << endl;
                //cout << "parent node.." << a->x << ", " << a->y << endl;
            }
    }
    cout << "dist2: " << tmpdist << ","<< mindist << endl;
    //cout << "push node.." << endl;
    //cout << "push node2.." << nNode->x << ", " << nNode->y << endl;
    
    
}
void InitNode()
{
    for(auto a : nodeList)
    {
        Node *remove = a;
        free(a);
    }
    nodeList = vector<Node*>();
}
int main()
{
    ofstream fsample;
    ofstream ftree;
    fsample.open("sampling.txt");
    ftree.open("tree.txt");
    std::random_device random_device;
    std::mt19937 engine(random_device());
    std::uniform_int_distribution<int> distribution(0.0, width);
    std::uniform_int_distribution<int> distribution2(0.0, height);
    double tmpdist;
    bool pathfound = false;
    Node *desNode = new Node;
    
    desNode->x = des_x;
    desNode->y = des_y;
    Node *head = new Node;
    Node *ptNode;
    Node *tmpparent;
    head->x = start_x;
    head->y = start_y;
    nodeList.push_back(head);
    objectList.push_back(make_pair(10,10));
    objectList.push_back(make_pair(20,20));
    objectList.push_back(make_pair(60,60));
    objectList.push_back(make_pair(78,82));
    //objectList.push_back(make_pair(100,100));
    objectList.push_back(make_pair(90,60));
    while(pathfound == false)
    {
        ptNode = nullptr;
        tmpparent = nullptr;
        ptNode = Sampling(distribution(engine), distribution2(engine));
       // cout << "ptNode: " << ptNode->x << "," << ptNode->y<< endl;
        if(collision(ptNode) == false)
            continue;
        //tmpNode = ptNode;
        conNode(ptNode);
        cout << ptNode->x << ", "<< ptNode->y << endl;
        fsample << ptNode->x << ", "<< ptNode->y << "\n";
        tmpparent = ptNode->parent;
        //cout <<"parentNode"<< tmpparent->x << ", "<< tmpparent->y << endl;
        tmpdist=sqrt(pow(desNode->x-ptNode->x,2)+pow(desNode->y-ptNode->y,2));
        //cout << "tmpdist: " << tmpdist << endl;
        
        if(tmpdist < 3)
            pathfound=true;

        
    }
    fsample.close();
    cout << "found! " << nodeList.size() << endl;
     cout << "count " << nodeList.size() << endl;
    
    while(pathfound==true)
    {
        Node *tmpNode = ptNode->parent;
        //cout <<ptNode->x<<", "<<ptNode->y<< endl;
        ftree << ptNode->x << ", "<< ptNode->y << "\n";
        tmpdist=sqrt(pow(nodeList[0]->x-ptNode->x,2)+pow(nodeList[0]->y-ptNode->y,2));
        if(tmpdist < 3)
            break;
        ptNode = tmpNode->parent;
        
    }
    ftree.close();
    InitNode();
    free(ptNode);
    return 0;

}
