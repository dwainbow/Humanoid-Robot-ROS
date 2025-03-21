// A map to store the Motor_Clusters for each body part.
std::map<Body_Part, std::shared_ptr<Motor_Cluster>> motor_clusters;

// To add a Motor_Cluster for each body part.
void create_motor_clusters(ros::NodeHandle &nh)
{
    motor_clusters[Body_Part::LEFT_ARM] = std::make_shared<Motor_Cluster>(nh, Body_Part::LEFT_ARM);
    motor_clusters[Body_Part::RIGHT_ARM] = std::make_shared<Motor_Cluster>(nh, Body_Part::RIGHT_ARM);
    motor_clusters[Body_Part::LEFT_LEG] = std::make_shared<Motor_Cluster>(nh, Body_Part::LEFT_LEG);
    motor_clusters[Body_Part::RIGHT_LEG] = std::make_shared<Motor_Cluster>(nh, Body_Part::RIGHT_LEG);
}
