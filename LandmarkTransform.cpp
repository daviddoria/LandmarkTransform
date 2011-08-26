#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <vtkLandmarkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

struct Coord3D
{
  float x,y,z;
  Coord3D() : x(0), y(0), z(0){}
};

std::vector<Coord3D> LoadPoints(const std::string& filename);
void Transform(vtkPolyData* source, vtkPolyData* destination, const std::vector<Coord3D> sourceCorrespondences, std::vector<Coord3D> destinationCorrespondences, vtkPolyData* output);

int main(int argc, char *argv[])
{
  //This program moves 'source' to 'destination', and saves the result as 'output'

  // Verify arguments
  if(argc != 6)
    {
    std::cerr << "Required arguments: Source.vtp Destination.vtp SourceCorrespondences.txt DestinationCorrespondences.txt Output.vtp" << std::endl;
    return EXIT_FAILURE;
    }

  // Parse arguments
  std::string sourceFileName = argv[1];
  std::string destinationFileName = argv[2];
  std::string sourceCorrespondencesFileName = argv[3];
  std::string destinationCorrespondencesFileName = argv[4];
  std::string outputFileName = argv[5];

  // Output arguments
  std::cout << "Source: " << sourceFileName << std::endl;
  std::cout << "Destination: " << destinationFileName << std::endl;
  std::cout << "Source correspondences: " << sourceCorrespondencesFileName << std::endl;
  std::cout << "Destination correspondences: " << destinationCorrespondencesFileName << std::endl;
  std::cout << "Output: " << outputFileName << std::endl;

  vtkSmartPointer<vtkXMLPolyDataReader> sourceReader =
    vtkSmartPointer<vtkXMLPolyDataReader>::New();
  sourceReader->SetFileName(sourceFileName.c_str());
  sourceReader->Update();

  vtkSmartPointer<vtkXMLPolyDataReader> destinationReader =
    vtkSmartPointer<vtkXMLPolyDataReader>::New();
  destinationReader->SetFileName(destinationFileName.c_str());
  destinationReader->Update();

  std::vector<Coord3D> sourceCorrespondences = LoadPoints(sourceCorrespondencesFileName);
  std::vector<Coord3D> destinationCorrespondences = LoadPoints(destinationCorrespondencesFileName);

  vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
  
  Transform(sourceReader->GetOutput(), destinationReader->GetOutput(), sourceCorrespondences, destinationCorrespondences, output);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInput(output);
  writer->Write();

  return EXIT_SUCCESS;
}

std::vector<Coord3D> LoadPoints(const std::string& filename)
{
  std::vector<Coord3D> points;
  
  std::string line;
  std::ifstream fin(filename.c_str());

  if(fin == NULL)
    {
    std::cerr << "Cannot open file." << std::endl;
    return points;
    }

  while(getline(fin, line))
    {
    std::stringstream ss;
    ss << line;
    Coord3D coord;
    ss >> coord.x >> coord.y >> coord.z;

    points.push_back(coord);
    }

  return points;
}

void Transform(vtkPolyData* source, vtkPolyData* destination, const std::vector<Coord3D> sourceCorrespondences, std::vector<Coord3D> destinationCorrespondences, vtkPolyData* output)
{
  if(sourceCorrespondences.size() != destinationCorrespondences.size())
    {
    std::cerr << "Number of sources correspondences is " << sourceCorrespondences.size()
              << " and number of destination correspondences is " << destinationCorrespondences.size()
              << " but they must match!" << std::endl;
    return;
    }

  unsigned int numberOfCorrespondences = sourceCorrespondences.size();
  
  vtkSmartPointer<vtkLandmarkTransform> landmarkTransform = vtkSmartPointer<vtkLandmarkTransform>::New();
  vtkSmartPointer<vtkPoints> sourcePoints = vtkSmartPointer<vtkPoints>::New(); //model points
  vtkSmartPointer<vtkPoints> targetPoints = vtkSmartPointer<vtkPoints>::New(); //scene points

  for(unsigned int i = 0; i < numberOfCorrespondences; ++i)
    {
    Coord3D sCoord = sourceCorrespondences[i];
    Coord3D dCoord = destinationCorrespondences[i];
    double sPoint[3] = {sCoord.x, sCoord.y, sCoord.z};
    double dPoint[3] = {dCoord.x, dCoord.y, dCoord.z};

    sourcePoints->InsertNextPoint(sPoint);
    targetPoints->InsertNextPoint(dPoint);
    }

  landmarkTransform->SetSourceLandmarks(sourcePoints);
  landmarkTransform->SetTargetLandmarks(targetPoints);
  landmarkTransform->SetModeToRigidBody();

  //transform the model by the calculated transform
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetInputConnection(source->GetProducerPort());
  transformFilter->SetTransform(landmarkTransform);
  transformFilter->Update();

  output->ShallowCopy(transformFilter->GetOutput());
}