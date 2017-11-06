# include "../include/wall_following.h"

int main(int argc,char **argv)
{
  ros::init(argc, argv, "stdr_wall_following", ros::init_options::AnonymousName);
  robo_stdr_reactive::WallFollowing obj(argc, argv);
  ros::spin();
  return 0;
}
