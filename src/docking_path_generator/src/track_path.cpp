#include <ros/ros.h>
#include <docking_path_generator/track_path.h>
#include <angles/angles.h>
//#include <tf/transform_datatypes.h>

namespace docking_path_generator
{
    TrackPath::TrackPath()
    {
    }

    std::vector<geometry_msgs::PoseStamped> TrackPath::adjust_path(std::vector<geometry_msgs::PoseStamped> &input)
    {
        std::vector<geometry_msgs::PoseStamped> output;
        double t;
        t = 0.1;
        geometry_msgs::PoseStamped goal_point = input[input.size() - 1];
        output = Bezier(t, input);
        output.push_back(goal_point);
        return output;
    }

    std::vector<geometry_msgs::PoseStamped> TrackPath::Bezier(double dt, std::vector<geometry_msgs::PoseStamped> &input)
    {
        std::vector<geometry_msgs::PoseStamped> output;
        if (input.size() == 3)
        {
            input.erase(input.begin() + 1);
        }

        double t = 0;
        while (t < 1) //t <= 1
        {
            geometry_msgs::PoseStamped p;
            double x_sum = 0.0;
            double y_sum = 0.0;
            int i = 0;
            int n = input.size() - 1;
            //int n = 3;
            while (i <= n)
            {
                double k = fac(n) / (fac(i) * fac(n - i)) * pow(t, i) * pow(1 - t, n - i);
                x_sum += k * input[i].pose.position.x;
                y_sum += k * input[i].pose.position.y;
                i++;
            }

            p.pose.position.x = x_sum;
            p.pose.position.y = y_sum;
            p.header = input[0].header;
            output.push_back(p);
            t += dt;
        }

        return output;
    }

    int TrackPath::fac(int x)
    {
        int f = 1;
        if (x == 0)
        {
            return f;
        }
        else
        {
            for (int i = 1; i <= x; i++)
            {
                f *= i;
            }

            return f;
        }
    }

}
