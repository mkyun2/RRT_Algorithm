#include <random>
#include <iostream>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core.hpp>
class RRT
{
    public:
        RRT()
        {}
        ~RRT()
        {}
        
        //std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        //nav2_costmap_2d::Costmap2D * costmap_;
        
        cv::Mat map;
        std::random_device random_device;
        double prev_time_;
        unsigned int sizeX_;
        unsigned int sizeY_;
        unsigned int startX_;
        unsigned int startY_;
        unsigned int goalX_;
        unsigned int goalY_;
        unsigned int collisionThr;
        
        double jump_;
        double resolution_;
        double sampleDistThr;
        double goalTolerance;

        unsigned int nodeSize;
        unsigned int batchSize_;
        
        int sampleRate_;
        struct Vertex {

            unsigned int x = 0;
            unsigned int y = 0;
            Vertex* parent = nullptr;
        };

        void InitVertex();
        double randSample(int size);
        bool Sampling(Vertex* sample, unsigned int randX, unsigned int randY);
        bool checkValidSample(Vertex* sample);
        void conNode(Vertex* nNode);
        void conVertex(Vertex* nVertex);
        bool checkFootPrint(int sampleX, int sampleY);
        void adjustGoal(double & goalThr);
        void configure(unsigned int startX, unsigned int startY, unsigned int goalX, unsigned int goalY);
        void createPlan();
        
};
int main();