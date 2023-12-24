#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <math.h>

using namespace std;
using namespace cv;
using namespace cv::aruco;

const std::string framesHolder = "experiments_with_ToF/glass_for_microscopes/";

const Size chessboardDimension = Size(10, 7);

bool getTransform = false;

void sendCamera2ImgStartPointTransform( const std::vector<std::vector<double>> &TranslationVectors,
                                        const std::vector<std::vector<double>> &RotationVectors,
                                        const size_t i){
  assert(TranslationVectors[i].size() == 3);
  assert(RotationVectors[i].size()    == 3);
  tf::TransformBroadcaster br;
  tf::Transform tr;
  tr.setOrigin( tf::Vector3(TranslationVectors[i][0], 
                            TranslationVectors[i][1], 
                            TranslationVectors[i][2]) );
  tf::Quaternion quaternion;
  quaternion.setRPY(RotationVectors[i][0], 
                    RotationVectors[i][1], 
                    RotationVectors[i][2]);
  tr.setRotation(quaternion);
  br.sendTransform( tf::StampedTransform(tr, ros::Time::now(), 
      "camera" + std::to_string(i), "img_start_point" + std::to_string(i)) );
}

void sendImgStartPoint2ImgSomePointTransform( const std::vector<std::vector<double>> &WorldPoints,
                                              const size_t i,
                                              const size_t j){
  assert(WorldPoints[j].size() == 2);
  tf::TransformBroadcaster br;
  tf::Transform tr;
  tr.setOrigin( tf::Vector3(  WorldPoints[j][0], 
                              WorldPoints[j][1], 
                              0) );
  tf::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0);
  tr.setRotation(quaternion);
  br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), 
      "img_start_point" + std::to_string(i), "img_some_point_" + std::to_string(i) + "_" + std::to_string(j)));
}

void getCamera2ImgSomePointTransform( const  tf::TransformListener &listener,
                                      tf::StampedTransform  &transform,
                                      const size_t i,
                                      const size_t j){
  try{
    listener.lookupTransform( "camera" + std::to_string(i), 
                              "img_some_point_" + std::to_string(i) + "_" + std::to_string(j),
                              ros::Time(0), transform);
    getTransform = true;
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    getTransform = false;
  }
}

void printCamera2ImgSomePointPose(const tf::StampedTransform &transform){
  tf::Quaternion quaternion(transform.getRotation().x(),
                            transform.getRotation().y(),
                            transform.getRotation().z(),
                            transform.getRotation().w());
  tf::Matrix3x3 m(quaternion);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  printf("\n");
  printf("x     = %f\n", transform.getOrigin().x());
  printf("y     = %f\n", transform.getOrigin().y());
  printf("z     = %f\n", transform.getOrigin().z());
  printf("roll  = %f\n", roll);
  printf("pitch = %f\n", pitch);
  printf("yaw   = %f\n", yaw);
}

void parseWord(const std::string &word, double &matWord){
  if(word.find('.') == std::string::npos){
    matWord = std::stod(word);
    return;
  }

  std::string integer    = word.substr(0, word.find('.'));
  std::string fractional = word.substr(word.find('.') + 1, word.size());
  std::string number     = integer + '.' + fractional;
  matWord                = std::stod(number);
}

void parseLine(const std::string &line, std::vector<double> &matLine){
  std::string word = "";
  double matWord   = 0;
  for (size_t i = 0; i < line.size(); i++)
  {
    if ((line[i] >= '0' && line[i] <= '9') || line[i] == '.' || line[i] == '-'){
      word += line[i];
      if (i + 1 == line.size()){
        parseWord(word, matWord);
        matLine.push_back(matWord); 
      }
    } else {
      parseWord(word, matWord);
      matLine.push_back(matWord);
      word = "";
    }
  }
}

void getMatFromFile(std::vector<std::vector<double>> &mat, std::string fileName){
  mat.clear();
  std::ifstream matFile(fileName);
  assert(matFile.is_open());
  std::string line = "";
  while (std::getline(matFile, line)){
    std::vector<double> matLine;
    parseLine(line, matLine);
    mat.push_back(matLine);
  }
}

void getVecFromFile(std::vector<double> &vec, std::string fileName){
  vec.clear();
  std::ifstream vecFile(fileName);
  assert(vecFile.is_open());
  std::string line = "";
  while (std::getline(vecFile, line)){
    parseLine(line, vec);
  }
}

void getCam2PointsZCoordinate(std::vector<double> &Cam2PointsZCoordinate,
                        const std::vector<std::vector<double>> &WorldPoints,
                        const std::vector<std::vector<double>> &TranslationVectors,
                        const std::vector<std::vector<double>> &RotationVectors,
                        const  tf::TransformListener &listener,
                        const size_t i){
  for (size_t j = 0; j < WorldPoints.size(); j++)       // для всех кл точек j на кадре i (54 шт)
  {
    ros::Rate r(100);
    tf::StampedTransform transform;
    do{
      sendCamera2ImgStartPointTransform(TranslationVectors, RotationVectors, i);
      sendImgStartPoint2ImgSomePointTransform(WorldPoints, i, j);
      getCamera2ImgSomePointTransform(listener, transform, i, j);
      r.sleep();
    } while(!getTransform);
    printCamera2ImgSomePointPose(transform);
    // Cam2PointsZCoordinate.push_back(std::sqrt(pow(transform.getOrigin().x(),2) + 
    //                                           pow(transform.getOrigin().y(),2) + 
    //                                           pow(transform.getOrigin().z(),2)));
    Cam2PointsZCoordinate.push_back(transform.getOrigin().z());

  }
}

void getFileFromVec(const std::vector<double> &vec, std::string fileName){
  assert(vec.size() == 54);
  std::ofstream vecFile(fileName);
  assert(vecFile.is_open());
  vecFile.clear();
  for (size_t i = 0; i < vec.size(); i++){
    vecFile << std::to_string(vec[i]) << std::endl;
  }
}

void getZWorldCoordForAllFrames(  const std::vector<std::vector<double>> &WorldPoints,
                                  const std::vector<std::vector<double>> &TranslationVectors,
                                  const std::vector<std::vector<double>> &RotationVectors,
                                  const  tf::TransformListener &listener){

  for (size_t i = 0; i < TranslationVectors.size(); i++)  // для всех кадров i < TranslationVectors.size() i - конкретный кадр
  {
    std::vector<double> Cam2PointsZCoordinate;            // Z для всех кл точек кадра
    getCam2PointsZCoordinate( Cam2PointsZCoordinate,
                              WorldPoints,
                              TranslationVectors,
                              RotationVectors,
                              listener,
                              i);
    assert(Cam2PointsZCoordinate.size() == 54);
    getFileFromVec(Cam2PointsZCoordinate, framesHolder + "Cam2PointsZ" + std::to_string(i) + ".txt");
  }
}

void printMat(std::vector<std::vector<double>> &mat){
  assert(!mat.empty());
  std::cout << "\n";
  for (size_t i = 0; i < mat.size(); i++)
  {
    for (auto iter = mat[i].begin(); iter != mat[i].end(); iter++)
    {
      printf("%f ", *iter);
    }
    std::cout << "\n";
  }
}

//извлечение из изображения обнаруженных углов шахматной доски (любые углы)
void getChessBoardCorners(const cv::Mat &img, std::vector<cv::Point2f> &foundCorners, const bool showResults, const size_t i){

  bool found = findChessboardCorners(img, Size(chessboardDimension.width - 1,chessboardDimension.height - 1), foundCorners,
                                      CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
  // std::cout << "found = " << found << std::endl;
  if (showResults) {
    drawChessboardCorners(img, Size(chessboardDimension.width - 1,chessboardDimension.height - 1), foundCorners, found);
    imshow(std::to_string(i), img);
  }
}

void printVecPoint2f(const std::vector<cv::Point2f> &vec){
  for (size_t i = 0; i < vec.size(); i++)
  {
    std::cout << std::round(vec[i].x) + 1 << "\t" << std::round(vec[i].y) + 1 << std::endl;
  }
}

void printVecDouble(const std::vector<double> &vec){
  for (size_t i = 0; i < vec.size(); i++)
  {
    std::cout << vec[i] << std::endl;
  }
}

void reverse(std::vector<cv::Point2f> &foundCorners){
  std::vector<cv::Point2f> tmpFoundCorners;

  for (size_t i = 0; i < 9; i++)
  {
    for (size_t j = 0; j < 6; j++)
    {
      tmpFoundCorners.push_back(foundCorners[j * 9 + i]);
    }
  }
  foundCorners.clear();
  foundCorners = tmpFoundCorners;

}

void getAllFoundCornersForAllFrames(const std::vector<std::vector<double>> &TranslationVectors,
                                              vector<vector<Point2f>> &allFoundCorners){

  for (size_t i = 0; i < TranslationVectors.size(); i++)  // для всех кадров i < TranslationVectors.size() i - конкретный кадр
  {
    // if (i == 13 || i == 22) continue;
    cv::Mat img;
    std::string imgAddr  = framesHolder + "m_depthFrame" + std::to_string(i) + ".png";
    img = cv::imread(imgAddr, cv::IMREAD_COLOR);
    assert(!img.empty());
    std::vector<cv::Point2f> foundCorners;
    getChessBoardCorners(img, foundCorners, 1, i);
    reverse(foundCorners);
    std::cout << "for frame " << std::to_string(i) << " foundCorners.size() = " << foundCorners.size() << std::endl;
    assert(foundCorners.size() == 54);
    allFoundCorners.push_back(foundCorners);
    // printVecPoint2f(foundCorners);
    waitKey(0);
  }
}

void getPopugayForOnePoint( const vector<vector<Point2f>> &allFoundCorners,
                            const std::vector<std::vector<double>> &m_depthFrame,
                            double &popugay,
                            const size_t &i,
                            const size_t &j){

  int row = std::round(allFoundCorners[i][j].y);
  int col = std::round(allFoundCorners[i][j].x);
  popugay = m_depthFrame[row][col];

  // allFoundCorners[i]    - вектор найденных углов на i кадре
  // allFoundCorners[i][j] - j угол на i кадре
  // allFoundCorners[i][j].x
  // allFoundCorners[i][j].y
}

void getPopugayVecForOneFrame(const vector<vector<Point2f>> &allFoundCorners, 
                                    vector<double> &popugayVec, 
                                    const size_t &i){
  
  std::vector<std::vector<double>> m_depthFrame;
  std::string fileAddr  = framesHolder + "m_depthFrame" + std::to_string(i) + ".txt";

  getMatFromFile(m_depthFrame, fileAddr);
  
  for (size_t j = 0; j < allFoundCorners[i].size(); j++)    // для всех точек j кадра i
  {
    double popugay = 0;
    getPopugayForOnePoint(allFoundCorners, m_depthFrame, popugay, i, j);
    popugayVec.push_back(popugay);
  }

}

void getDeffKoeffVecForOneFrame(const vector<vector<Point2f>> &allFoundCorners,
                                      vector<double> &deffKoeffVec, 
                                      const size_t &i){

  vector<double> PopugayVec;                                // попугаи для всех кл точек кадра
  vector<double> Cam2PointsZCoordinate;                     // Z для всех кл точек кадра

  std::string PopugayVecFileAddr            = framesHolder + "PopugayVec" + std::to_string(i) + ".txt";
  std::string Cam2PointsZCoordinateFileAddr = framesHolder + "Cam2PointsZ" + std::to_string(i) + ".txt";

  getVecFromFile(PopugayVec, PopugayVecFileAddr);
  getVecFromFile(Cam2PointsZCoordinate, Cam2PointsZCoordinateFileAddr);

  assert(PopugayVec.size() == 54);
  assert(Cam2PointsZCoordinate.size() == 54);

  // std::cout << "PopugayVec:\n";
  // printVecDouble(PopugayVec);
  // std::cout << "Cam2PointsZCoordinate:\n";
  // printVecDouble(Cam2PointsZCoordinate);

  for (size_t j = 0; j < allFoundCorners[i].size(); j++)    // для всех точек j кадра i
  {
    deffKoeffVec.push_back(PopugayVec[j] / Cam2PointsZCoordinate[j]);
  }

}

void getPopugaysForAllFrames(const vector<vector<Point2f>> &allFoundCorners){

  for (size_t i = 0; i < allFoundCorners.size(); i++)       // для всех кадров i < allFoundCorners.size() i - конкретный кадр
  {
    // if (i == 13 || i == 22) continue;
    vector<double> PopugayVec;                              // попугаи для всех кл точек кадра
    getPopugayVecForOneFrame(allFoundCorners, PopugayVec, i);
    assert(PopugayVec.size() == 54);
    getFileFromVec(PopugayVec, framesHolder + "PopugayVec" + std::to_string(i) + ".txt");
  }
}

void getDeffKoeffForAllFrames(const vector<vector<Point2f>> &allFoundCorners){

  for (size_t i = 0; i < allFoundCorners.size(); i++)       // для всех кадров i < allFoundCorners.size() i - конкретный кадр
  {
    // if (i == 13 || i == 22) continue;
    vector<double> DeffKoeffVec;                            // коэффициенты для всех кл точек кадра
    getDeffKoeffVecForOneFrame(allFoundCorners, DeffKoeffVec, i);
    assert(DeffKoeffVec.size() == 54);
    getFileFromVec(DeffKoeffVec, framesHolder + "DeffKoeffVecZ" + std::to_string(i) + ".txt");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pointTransform");
  ros::NodeHandle n;

  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  std::vector<std::vector<double>>  WorldPoints;
  std::vector<std::vector<double>>  TranslationVectors;
  std::vector<std::vector<double>>  RotationVectors;
  getMatFromFile(WorldPoints,        framesHolder + "WorldPoints.txt");
  getMatFromFile(TranslationVectors, framesHolder + "TranslationVectors.txt");
  getMatFromFile(RotationVectors,    framesHolder + "RotationVectors.txt");

  // printf("\nWorldPoints:\n");
  // printMat(WorldPoints);
  // printf("\nTranslationVectors:\n");
  // printMat(TranslationVectors);
  // printf("\nRotationVectors:\n");
  // printMat(RotationVectors);

  getZWorldCoordForAllFrames(WorldPoints, TranslationVectors, RotationVectors, listener);

  vector<vector<Point2f>> allFoundCorners;  // i х 54 углов шахматной доски
  getAllFoundCornersForAllFrames(TranslationVectors, allFoundCorners);
  std::cout << "allFoundCorners.size() = " << allFoundCorners.size() << std::endl;

  getPopugaysForAllFrames(allFoundCorners);
  getDeffKoeffForAllFrames(allFoundCorners);

  return 0;
}