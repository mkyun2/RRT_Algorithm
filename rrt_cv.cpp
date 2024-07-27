#include <iostream>
#include "include/rrt_cv.hpp"
using namespace std;

#define thrhold 5
#define distance_object_to_line 0.1



vector<RRT::Vertex*> nodeList;
int main()
{
    unsigned int startX, startY, goalX, goalY;
    RRT Planner;
    while(true)
    {
        printf("Input startX startY \n");
        scanf("%d %d",&startX,&startY);
        printf("Input goalX goalY \n");
        scanf("%d %d",&goalX,&goalY);
        Planner.configure(startX,startY,goalX,goalY);
        Planner.createPlan();
    }
    return 0;
}
double RRT::randSample(int size)
{
    std::random_device random_device;
    std::mt19937 engine(random_device());
    std::uniform_int_distribution<int> distribution(0.0, size);

    return distribution(engine);

}
bool RRT::Sampling(Vertex* sample, unsigned int randX, unsigned int randY) 
{
    //Node *sample = new Node;// Random Sample
    //Node *tmpNode;
    double dist = 0.0;
    double angle =0.0;
    double mindist = 1000.0;
    int i = -1;
    int index = 0;
    
    sample->x = randX;//randSample(width);
    sample->y = randY;//randSample(height);
    
    //cout <<"sample:" << sample->x << ", " << sample->y << endl;
    bool flagOfValidSample = false;
    
    for (auto a : nodeList)
    {
        i++;
        unsigned int nodeSampleX = a->x;
        unsigned int nodeSampleY = a->y;
        dist = hypot((double)nodeSampleX - (double)randX,(double)nodeSampleY- (double)randY); // Calc dist to find minimum Qnear to Qrand 
        angle = atan2f(((int)randY- (int)nodeSampleY), ((int)randX - (int)nodeSampleX)); //x cosangle y sin angle
        //cout <<"angle:" << angle <<"dist: "<< dist <<  endl;
        //cout << "diff (" << randX - nodeSampleX << " ," << randY - nodeSampleY << ")"<< endl; 
        if (dist <= mindist && dist >= sampleDistThr/resolution_)
        {
            mindist = dist;
            index = i;
            flagOfValidSample = true;
        }
        else if(dist > mindist || dist < sampleDistThr/resolution_)
        {
            continue;
        }
    }
    if(!flagOfValidSample)
        {
            free(sample);
            return false;
        }
    //Node *nNode = new Node;// New node between Qnear and Qrand
    // nNode -> x = nodeList[index]->x + jump*cosf(angle);
    // nNode -> y = nodeList[index]->y + jump*sinf(angle);
    // nNode -> parent = nodeList[index];
    ///////////generation New Node//////////////////////////
    unsigned int newSampleX = (unsigned int)((double)nodeList[index]->x + jump_ * cosf(angle));
    unsigned int newSampleY = (unsigned int)((double)nodeList[index]->y + jump_ * sinf(angle));
    // if(seen_[newSampleX][newSampleY])
    // {
    //     cout <<"Seen!" << newSampleX << ", " << newSampleY << endl;
    //     free(sample);
    //     return false;
    // } 
    sample->x = newSampleX;
    sample->y = newSampleY;
    sample->parent = nodeList[index];
    //cout <<"index:" << index << ", " << nodeList[index]->x << ", "<< nodeList[index]->x<< endl;


    return true;

}
bool RRT::checkFootPrint(int sampleX, int sampleY)
{
    int footPrintX = std::abs((int)startX_ - sampleX);
    int footPrintY = std::abs((int)startY_ - sampleY);

    if(footPrintX <=2 && footPrintY <=2 )
        return true;
    else
        return false;
}
bool RRT::checkValidSample(Vertex* sample)
{
    Vertex* parent = sample->parent;
    double lineGrad;
    unsigned int tmpX, tmpY;
    int sampleX = (int)sample->x;
    int sampleY = (int)sample->y;
    int parentX = (int)parent->x;
    int parentY = (int)parent->y;
    int diffX = (int)((sampleX - parentX));
    int diffY = (int)((sampleY - parentY)); 

    if (diffX != 0)
        lineGrad = ((double)diffY / (double)diffX);
    else
        lineGrad = 0.0;

    //interceptY = round(parentY - lineGrad * parentX);
    if (sampleX >= (int)sizeX_ || sampleX <= 0)
    {
        free(sample);
        return false;
    }
    else if (sampleY >= (int)sizeY_ || sampleY <= 0)
    {
        free(sample);
        return false;
    }
    
    //unsigned int index2 = costmap_->getIndex(sample->x,sample->y);
    //unsigned int getCost2 = costmap_->getCost(index2);
    unsigned char getCost2 = map.at<uchar>(sample->y,sample->x);
    //cout << " Check 1" << endl;
    if(getCost2 > collisionThr && !checkFootPrint(sample->x,sample->y))
    {
        //cout << " why2" << endl;
        free(sample);
        //cout << " why3" << endl;
        return false;
    }
    if (diffX == 0)
    {
        //cout << " Check 2" << endl;
        //cout<<"objectX:" << a.first << ", objectY:" << a.second << endl;
        //cout<<"sampleX:" << sample->x << ", sampleY:" << sample->y << endl;
        //cout<<"parentX:" << parent->x << ", parentY:" << parent->y << endl;
        //cout<<"grad:" << lineGrad << ", intercept:" << interceptY << endl;
        if (parentY < sampleY) 
        {
            for (int y1 = parentY; y1 < (sampleY); y1++)
            {
                tmpX = parentX;
                tmpY = y1;
                //unsigned int index = costmap_->getIndex(tmpX,tmpY);
                //unsigned int getCost = costmap_->getCost(index);
                unsigned char getCost = map.at<uchar>(tmpY,tmpX);
                if(getCost > collisionThr)
                {
                    free(sample);
                    return false;
                }
            }
        }
        else 
        {
            for ( int y1 = parentY; y1 > (sampleY); y1--)
            {
                
                tmpX = parentX;
                tmpY = y1;
                //unsigned int index = costmap_->getIndex(tmpX,tmpY);
                unsigned char getCost = map.at<uchar>(tmpY,tmpX);
                if(getCost > collisionThr)
                {
                    free(sample);
                    return false;
                }
            }
        }
    }
    else
    {
        //cout << " Check 3" << endl;
        if (sampleX > parentX && sampleY >= parentY) // 1
        {
            for ( int x1 = 0; x1 < diffX; x1++)
            {
                tmpX = (unsigned int)(parentX+x1);
                tmpY = (unsigned int)(lineGrad * (double)x1)+(unsigned int)parentY;
                //std::cout << lineGrad <<" **"<< x1 << "+"<<parentY <<"="<< tmpY << std::endl;
                //unsigned int index = costmap_->getIndex(tmpX,tmpY);
                unsigned char getCost = map.at<uchar>(tmpY,tmpX);
                if(getCost > collisionThr && !checkFootPrint(tmpX,tmpY))
                {
                    //std::cout << " Check 3-1" << std::endl;
                    std::cout << tmpX << ", " << tmpY << ", " << getCost << std::endl;
                    std::cout <<lineGrad<<","<< parentX << ", " << sampleX << ", " << parentY << ", " << sampleY << std::endl;
                    free(sample);
                    return false;
                }
            }
        }
        else if (sampleX < parentX && sampleY > parentY) // 2
        {
            for ( int x1 = diffX; x1 < 0; x1++)
            {
                tmpX = (unsigned int)(parentX+x1);
                tmpY = (unsigned int)(lineGrad * (double)x1) + (unsigned int)parentY;
                //std::cout << lineGrad <<" **"<<(double)(x1/5.0) << "+"<<parentY <<"="<< tmpY << std::endl;
                //unsigned int index = costmap_->getIndex(tmpX,tmpY);
                unsigned char getCost = map.at<uchar>(tmpY,tmpX);
                if(getCost > collisionThr && !checkFootPrint(tmpX,tmpY))
                {
                    //cout << " Check 3-2" << endl;
                    std::cout << tmpX << ", " << tmpY << ", " << getCost << std::endl;
                    std::cout <<lineGrad<<","<< parentX << ", " << sampleX << ", " << parentY << ", " << sampleY << std::endl;
                    free(sample);
                    return false;
                }
            }
        }
        else if (sampleX > parentX && sampleY < parentY) // 3
        {
            for ( int x1 = 0; x1 < diffX; x1++)
            {
                tmpX = (unsigned int)(parentX+x1);
                tmpY = (unsigned int)(lineGrad * (double)x1) + (unsigned int)parentY;
                //std::cout << lineGrad <<" **"<<x1 << "+"<<parentY <<"="<< tmpY << std::endl;
                //unsigned int index = costmap_->getIndex(tmpX,tmpY);
                unsigned char getCost = map.at<uchar>(tmpY,tmpX);
                if(getCost > collisionThr && !checkFootPrint(tmpX,tmpY))
                {
                    //cout << " Check 3-3" << endl;
                    std::cout << tmpX << ", " << tmpY << ", " << getCost << std::endl;
                    std::cout <<lineGrad<<","<< parentX << ", " << sampleX << ", " << parentY << ", " << sampleY << std::endl;
                    free(sample);
                    return false;
                }
            }
        }
        else if(sampleX < parentX && sampleY < parentY)// 4
        {
            for ( int x1 = diffX; x1 < 0; x1++)
            {
                tmpX = (unsigned int)(parentX+x1);
                tmpY = (unsigned int)(lineGrad * (double)x1) + (unsigned int)parentY;
                
                //std::cout << lineGrad <<" **"<<x1 << "+"<<parentY <<"="<< tmpY << std::endl;
                //unsigned int index = costmap_->getIndex(tmpX,tmpY);
                //std::cout << tmpX <<", "<< tmpY<< "getIndex: "<< index << std::endl;
                unsigned char getCost = map.at<uchar>(tmpY,tmpX);
                //std::cout << "getCosts"<< std::endl;
                if(getCost > collisionThr && !checkFootPrint(tmpX,tmpY))
                {
                    //cout << " Check 3-4" << endl;
                    std::cout << tmpX << ", " << tmpY << ", " << getCost << std::endl;
                    std::cout <<lineGrad<<","<< parentX << ", " << sampleX << ", " << parentY << ", " << sampleY << std::endl;
                    free(sample);
                    return false;
                }
            }
        }
    }
    //cout << " Check complete" << endl;
    return true;


}
void RRT::conNode(Vertex* nNode)
{
    double mindist = 1000.0;
    double tempDist = 1000.0;
    Vertex* tmpNode = nNode;
    //새롭게 연결된 노드
    for (auto a : nodeList)
    {

        tempDist = hypot((double)a->x - (double)tmpNode->x,(double)a->y - (double)tmpNode->y);
        //cout << "dist.." << tempDist << ","<< mindist << endl;
        //cout << "dist2.." << tempDist << ","<< mindist << endl;
        if (tempDist < thrhold)
            return;
        if (tempDist < mindist)
        {
            tmpNode->parent = nullptr;

            //cout << "dist2.." << tempDist << ","<< mindist << endl;
            mindist = tempDist;
            tmpNode->parent = a;
            //cout << "nNode node.." << nNode->x << ", " << nNode->y << endl;
            //cout << "parent node.." << a->x << ", " << a->y << endl;
        }
    }
    //cout << "dist2: " << tempDist << ","<< mindist << endl;
    //cout << "push node.." << endl;
    //cout << "push node2.." << nNode->x << ", " << nNode->y << endl;


}
void RRT::InitVertex()
{
    for (auto a : nodeList)
    {
        Vertex* remove = a;
        free(remove);
    }
    nodeList = vector<Vertex*>();
}
void RRT::configure(unsigned int startX, unsigned int startY, unsigned int goalX, unsigned int goalY)
{
    sizeX_ = 300;
    sizeY_= 300;
    resolution_ = 0.05;
    jump_ = 3;
    startX_ = startX;
    startY_ = startY;
    goalX_ = goalX;
    goalY_ = goalY;
    sampleDistThr = 0.1;
    nodeSize = 0;
    goalTolerance = 10;
    collisionThr = 1;
    map = cv::Mat::zeros(sizeY_,sizeX_,CV_8UC1);
    printf("Set Map Size(%d, %d)\n",sizeX_,sizeY_);
    printf("Set start point(%d, %d) and goal point(%d, %d)\n",startX_,startY_,goalX_,goalY_);
}
void RRT::adjustGoal(double & goalThr)
{
    if(nodeSize < nodeList.size())
    {
        goalThr = goalThr + 5;
    }
    nodeSize = nodeList.size();
}
void RRT::createPlan()
{
    double tempDist;
    bool result;
    //std::vector<nav_msgs::msg::Path> samplePaths;

    std::mt19937 engine(random_device());
    
    std::uniform_int_distribution<unsigned int> distribution(0.0, sizeX_-1);
    std::uniform_int_distribution<unsigned int> distribution2(0.0, sizeY_-1);
    //Define StartPose
    bool validPathFlag = false;
    Vertex* head = new Vertex;
    Vertex* ptNode = nullptr;
    head->x = startX_;
    head->y = startY_;

    nodeList.push_back(head);
    while (validPathFlag == false)
    {
        ptNode = new Vertex;
        if (!Sampling(ptNode, distribution(engine), distribution2(engine)))
            continue;
        if (!checkValidSample(ptNode))
            continue;
        unsigned int sampleX = ptNode->x;
        unsigned int sampleY = ptNode->y;
        nodeList.push_back(ptNode);
        //printf("new Node!");
        tempDist = hypot(goalX_ - sampleX, goalY_ - sampleY);
        if (tempDist < goalTolerance)
            validPathFlag = true;
        else if(nodeList.size()>sizeX_*sizeY_ && !validPathFlag)
            adjustGoal(goalTolerance);
    }
    vector<std::pair<int,int>> foundPlan;
    while(validPathFlag == true)
    {
        foundPlan.push_back({ptNode->y,ptNode->x});
        map.at<uchar>(ptNode->y,ptNode->x) = 254;
        //printf("(%d,%d) \n",ptNode->x,ptNode->y);
        if(ptNode->parent == nullptr)
            break;
        Vertex* tmpNode = ptNode->parent;            
        //tempDist = sqrt(pow(nodeList[0]->x - ptNode->x, 2) + pow(nodeList[0]->y - ptNode->y, 2));
        //printf("(%d,%d)",ptNode->x,ptNode->y);
        ptNode = tmpNode;
    }
    
    printf("Found Plan. Plan Size %ld \n",foundPlan.size());
    cv::imshow("Map",map);
    std::cout << "press 'q' Key on cv Window" << std::endl;
    int key = cv::waitKey();
    if(key == 'q')
    {
        std::cout << "Exit"<< std::endl;
        cv::destroyAllWindows();
    }
    InitVertex();

}