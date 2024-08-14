#include "Perception.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/////////////////////////////////////
/// CONSTRUTOR e FUNCOES PUBLICAS ///
/////////////////////////////////////

Perception::Perception(ros::NodeHandle& n):
    nh_(n)
{
    started_ = false;
    diffAngleToGradientDescent_ = 0.0;
    validGradientDescent_=false;


    // Initialize transform listener
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

    // Initialize publishers
    pub_mapBoundaries_ = nh_.advertise<nav_msgs::OccupancyGrid>("/mapa_boundaries", 1);
    pub_mapPotField_ = nh_.advertise<nav_msgs::OccupancyGrid>("/mapa_pot_field", 1);
    pub_gradientDescent_ = nh_.advertise<geometry_msgs::PoseStamped>("/pot_gradientDescent", 1);
 
    // Initialize subscribers
    sub_gridmap_ = nh_.subscribe("/mapa_laser_HIMM", 1, &Perception::receiveGridmap, this);

}

bool Perception::hasValidGradientDescent()
{
    if(validGradientDescent_==false)
        std::cout << "Gradiente descendente nulo" << std::endl;
    return validGradientDescent_;
}

double Perception::getDiffAngleToGradientDescent()
{
    return diffAngleToGradientDescent_;
}

/////////////////////////////////////
/// Callback dos topicos de MAPA  ///
/////////////////////////////////////

void Perception::receiveGridmap(const nav_msgs::OccupancyGrid::ConstPtr &value)
{
    // STRUCTURE OF nav_msgs::OccupancyGrid
    // # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    // Header header 
    // 
    // # MetaData for the map
    //   # The time at which the map was loaded
    //   time map_load_time
    //   # The map resolution [m/cell]
    //   float32 resolution
    //   # Map width [cells]
    //   uint32 width
    //   # Map height [cells]
    //   uint32 height
    //   # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
    //   geometry_msgs/Pose origin
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).  
    // # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    // # OBS: implemented in c++ with std::vector<u_int8>
    // int8[] data

    if(started_==false){
        numCellsX_ = value->info.width;
        numCellsY_ = value->info.height;

        float cellSize = value->info.resolution;

        mapWidth_ = numCellsX_*cellSize;
        mapHeight_ = numCellsY_*cellSize;
        scale_ = 1.0/cellSize;

        boundariesGrid_.resize(numCellsX_*numCellsY_,0);
        potFieldGrid_.resize(numCellsX_*numCellsY_,0.0);
        msg_potField_.data.resize(numCellsX_*numCellsY_,0);

        minKnownX_ = numCellsX_-1;
        minKnownY_ = numCellsY_-1;
        maxKnownX_ = maxKnownY_ = 0;

        started_=true;
    }

    for(unsigned int i=0; i<numCellsX_*numCellsY_; i++)
        boundariesGrid_[i] = value->data[i];

    updateGridKnownLimits();
    updateCellsClassification();
    for(int k=0;k<200;k++)
        updatePotentialField();

    msg_boundaries_.header = value->header;
    msg_boundaries_.info = value->info;
    msg_boundaries_.data = boundariesGrid_;

    msg_potField_.header = value->header;
    msg_potField_.info = value->info;
    for(unsigned int i=0; i<numCellsX_*numCellsY_; i++)
        if(boundariesGrid_[i]==-1)
            msg_potField_.data[i]=-1;
        else
            msg_potField_.data[i] = (int)round(potFieldGrid_[i]*100.0);

    msg_gradientDescent_.header = value->header;

    Pose2D robotPose = getCurrentRobotPose();
    msg_gradientDescent_.pose.position.x = robotPose.x;
    msg_gradientDescent_.pose.position.y = robotPose.y;
    msg_gradientDescent_.pose.position.z = 0;

    double yaw = computeDirectionOfGradientDescent(robotPose);
    diffAngleToGradientDescent_ = normalizeAngleDEG(RAD2DEG(yaw)-robotPose.theta);

    tf2::Quaternion quat_tf;
    quat_tf.setRPY( 0, 0, yaw );
    msg_gradientDescent_.pose.orientation=tf2::toMsg(quat_tf);
    
    pub_mapBoundaries_.publish(msg_boundaries_);
    pub_mapPotField_.publish(msg_potField_);
    pub_gradientDescent_.publish(msg_gradientDescent_);
}

/////////////////////////////////////////////////
/// FUNCOES DE COMPUTACAO DO CAMPO POTENCIAL  ///
/////////////////////////////////////////////////

void Perception::updateCellsClassification()
{
    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa 'boundariesGrid_'
    /// e atualize os valores, marcando como condição de contorno se for o caso

    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    unsigned int i = x + y*numCellsX_

    /// grid na entrada (valor inteiro): 
    /// - celulas desconhecidas = -1 
    /// - celulas conhecidas com ocupação variando de 0 a 100

    /// grid na saida (valor inteiro):
    /// - celulas desconhecidas seguem -1
    /// - celulas de obstaculo = 100
    /// - celulas de fronteira = 0
    /// - celulas livres vizinhas a obstaculos = 90
    /// - demais celulas livres = 50

    int dangerZoneWidth = 1;

    for(int x = minKnownX_; x <= maxKnownX_; x++) {
      for(int y = minKnownY_; y <= maxKnownY_; y++) {
        int i = x + y*numCellsX_;

        if(boundariesGrid_[i] == -1) {
          boundariesGrid_[i] = OCC_UNEXPLORED;
        } else if(boundariesGrid_[i] == 100) {
          boundariesGrid_[i] = OCC_OCCUPIED;
          potFieldGrid_[i] = 1.0;
          for(int auxX = dangerZoneWidth * -1; auxX<=dangerZoneWidth; auxX++) {
            for(int auxY = dangerZoneWidth * -1; auxY<=dangerZoneWidth; auxY++) {
              int i_aux = (x+auxX) + (y+auxY)*numCellsX_;
              if(boundariesGrid_[i_aux] != OCC_OCCUPIED)
                boundariesGrid_[i_aux] = OCC_NEAROBSTACLE;
                potFieldGrid_[i] = 1.0;
            }
          }
        } else if(boundariesGrid_[i] != OCC_OCCUPIED &&
                  boundariesGrid_[i] != OCC_NEAROBSTACLE) {

          if(boundariesGrid_[i] != OCC_UNEXPLORED)
            boundariesGrid_[i] = OCC_FREE;
          
          for(int auxX = -1; auxX<=1; auxX++) {
            for(int auxY = -1; auxY<=1; auxY++) {
              int i_aux = (x+auxX) + (y+auxY)*numCellsX_;
              if(boundariesGrid_[i_aux] == OCC_UNEXPLORED && boundariesGrid_[i] == OCC_FREE) {
                boundariesGrid_[i] = OCC_FRONTIER;
                potFieldGrid_[i] = 0.0;
              }
            }
          }
        }
      }
    }

}

void Perception::updatePotentialField()
{
    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa
    /// e atualize os valores em 'potFieldGrid_' (que é float), 
    /// consultando a classificação das celulas em 'boundariesGrid_' (que é inteiro)

    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    unsigned int i = x + y*numCellsX_

    /// grid na saida ('potFieldGrid_'):
    /// - celulas desconhecidas: nao computa potencial
    /// - celulas de fronteira: potencial fixo 0.0
    /// - celulas de obstaculo ou vizinhas: potencial fixo 1.0
    /// - demais celulas livres: computa potencial em funcao dos vizinhos

    for(int x = minKnownX_; x <= maxKnownX_; x++) {
      for(int y = minKnownY_; y <= maxKnownY_; y++) {
        int i = x + y*numCellsX_;
        
        int up = y+1;
        int down = y-1;
        int right = x+1;
        int left = x-1;

        if(boundariesGrid_[i] == OCC_OCCUPIED || boundariesGrid_[i] == OCC_NEAROBSTACLE) {
          potFieldGrid_[i] = 1.0;
        } else if (boundariesGrid_[i] == OCC_FRONTIER) {
          potFieldGrid_[i] = 0.0;
        } else {
          int i_aux_up = x + up*numCellsX_;
          int i_aux_down = x + down*numCellsX_;
          int i_aux_right = right + y*numCellsX_;
          int i_aux_left = left + y*numCellsX_;

          float potFieldAux = potFieldGrid_[i_aux_up] +
                                potFieldGrid_[i_aux_down] +
                                potFieldGrid_[i_aux_right] +
                                potFieldGrid_[i_aux_left];

          potFieldGrid_[i] = potFieldAux/4;

          if(potFieldGrid_[i] > 1) {
            potFieldGrid_[i] = potFieldGrid_[i]/100;
          }
        }
      }
    }

}

double Perception::computeDirectionOfGradientDescent(Pose2D robot)
{
    int rx = robot.x*scale_ + numCellsX_/2;
    int ry = robot.y*scale_ + numCellsY_/2;

    /// TODO: 
    /// computar direcao do gradiente descendente na celula do robô
    /// usando potenciais dos quatro vizinhos
    /// para acessar o indice da celula do robo usar:
    /// unsigned int i = rx + (ry)*numCellsX_;

    /// compute a diferença de potencial em x e y e calcule o angulo 'yaw' usando atan2
    /// resultado é em radianos
    double yaw=0;
    double dx=0.0, dy=0.0;

    int up = ry+1;
    int down = ry-1;
    int right = rx+1;
    int left = rx-1;

    int i_aux_up = rx + up*numCellsX_;
    int i_aux_down = rx + down*numCellsX_;
    int i_aux_right = right + ry*numCellsX_;
    int i_aux_left = left + ry*numCellsX_;

    dx = ((potFieldGrid_[i_aux_right]-potFieldGrid_[i_aux_left]))/2;
    dy = ((potFieldGrid_[i_aux_up]-potFieldGrid_[i_aux_down]))/2;

    yaw = atan2(-dy,-dx);

    // teste para verificar se gradiente é valido ou nulo
    if(fabs(dx)<1e-30 && fabs(dy)<1e-30)
        validGradientDescent_=false;
    else
        validGradientDescent_=true;

    return yaw;
}

///////////////////////////
/// FUNCOES AUXILIARES  ///
///////////////////////////

void Perception::updateGridKnownLimits()
{
    for(unsigned int x=0; x<numCellsX_; x++){
        for(unsigned int y=0; y<numCellsY_; y++){
            unsigned int i = x + y*numCellsX_;
            if(boundariesGrid_[i]!=-1)
            {
                if(x<minKnownX_) minKnownX_ = x;
                if(y<minKnownY_) minKnownY_ = y;
                if(x>maxKnownX_) maxKnownX_ = x;
                if(y>maxKnownY_) maxKnownY_ = y;
            }
        }
    }
}

Pose2D Perception::getCurrentRobotPose()
{
    Pose2D robotPose;

    // Get robot transformation given by ODOM
    geometry_msgs::TransformStamped transformStamped;
    try{ transformStamped = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_WARN("%s",ex.what()); }
    
    robotPose.x = transformStamped.transform.translation.x;
    robotPose.y = transformStamped.transform.translation.y;

    // Convert quaternion to euler angles
    tf2::Quaternion q4(transformStamped.transform.rotation.x,
                       transformStamped.transform.rotation.y, 
                       transformStamped.transform.rotation.z, 
                       transformStamped.transform.rotation.w);
    tf2::Matrix3x3 m4(q4);
    double roll, pitch, yaw;
    m4.getRPY(roll,pitch,yaw);

    robotPose.theta = RAD2DEG(yaw);

    return robotPose;
}
