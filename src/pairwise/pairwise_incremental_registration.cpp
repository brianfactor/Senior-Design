/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* Adapted from a variety of PCL tutorials http://pointclouds.org/documentation/tutorials/
 * adaptation Taylor Phebus*/
//This works well:
//time ./pairwise_incremental_registration 3.ply 5.ply 7.ply 9.ply 11.ply 13.ply 15.ply 17.ply 19.ply 21.ply 23.ply 25.ply .7 22 10.5 200 1 .3 outTestRadius.ply
#include <cmath>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//#include <pcl/registration/ndt.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>


using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
	//our visualizer
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".ply");
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc-7; i++)
  {
    std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPLYFile (argv[i], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}



/* ---[ */
int main (int argc, char** argv)
{
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);

  // Check user input
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s rotate.ply concatonate.ply angle destination.ply", argv[0]);
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());
  
    //Eigen::Matrix4f rotate30 = Eigen::Matrix4f::Identity ();//, pairTransform;

  std::cerr << "Begin Transform"<<endl;	
    PointCloud::Ptr result (new PointCloud), source, dest, target=data[1].cloud;
    //argc-2 =angle
    //double ang=atof(argv[argc-2])
    double ang=-30*3.14159/180;
    Eigen::Affine3f transform=pcl::getTransformation(atof(argv[argc-6]),0,atof(argv[argc-5]),0,ang,0);
    Eigen::Matrix4f rotate30 = transform.matrix();
    //Begin temp
    std::cout<<"Transform"<<endl;
    ang=-20*3.14159/180;
    ang=0*3.14159/180;
    Eigen::Affine3f transformTilt=pcl::getTransformation(0,0,0,ang,0,0);
    Eigen::Matrix4f rotateTilt = transformTilt.matrix();
    for(int i=0; i<12; i++){
	pcl::transformPointCloud(*(data[i].cloud), *(data[i].cloud),rotateTilt);
    }
    /*target=data[0].cloud;
    pcl::transformPointCloud (*target, *target,rotateTilt);	
    std::cout<<"Save"<<endl;
    std::stringstream s1;
    s1 << argv[argc-1];
    std::cout<<"Save "<<endl;
    pcl::io::savePLYFile (s1.str (), *target, true);*/
   //end temp
    dest=data[0].cloud;
   for(int i=1; i<12; i++){
	//if(i!=11){continue;}
        std::cerr<<"Rotate "<<i<<endl;
	
	target=data[i].cloud;
	for(int j=0; j<i; j++){
    		pcl::transformPointCloud (*target, *target,rotate30);	
	}
    	(*(dest))+=(*target);
    }


    //Stitching complete, now downsampling
  std::cerr << "PointCloud before downsampling: " << dest->width * dest->height 
       << " data points (" << pcl::getFieldsList (*dest) << ").";	
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (dest);

    //double ang=atof(argv[argc-2]);
  sor.setLeafSize (
atof(argv[argc-2]),
atof(argv[argc-2]),
atof(argv[argc-2]));
  if(atof(argv[argc-2])!=0){sor.filter (*dest);}

  std::cerr << "PointCloud after downsampling: " << dest->width * dest->height 
       << " data points (" << pcl::getFieldsList (*dest) << ").";	

  std::cerr << "PointCloud before filtering: " << dest->width * dest->height 
       << " data points (" << pcl::getFieldsList (*dest) << ").";	
	//Limit filter.
  pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices
      ptfilter.setInputCloud (dest);
      ptfilter.setFilterFieldName ("y");
      ptfilter.setFilterLimits (-20.0, 0.0);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr FiltTemp (new pcl::PointCloud<pcl::PointXYZ>);
      ptfilter.filter (*dest);
      // The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
      // The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
      // and also indexes all non-finite points of cloud_in
      ptfilter.setInputCloud (dest);
      ptfilter.setFilterFieldName ("x");
      ptfilter.setFilterLimits (-20.0, 0.0);
      ptfilter.setNegative (false);
      ptfilter.filter (*dest);
      // The resulting cloud_out contains all points of cloud_in that are finite and have:
      // x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and intensity smaller than 0.5.
	

	//Statistical filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> st;
  st.setInputCloud (dest);
  st.setMeanK (atoi(argv[argc-4]));
  st.setStddevMulThresh (atof(argv[argc-3]));
  st.filter (*dest);
  std::cerr << "PointCloud after filtering: " << dest->width * dest->height 
       << " data points (" << pcl::getFieldsList (*dest) << ").";	


  //writer.write<pcl::PointXYZ> ("inliers_"+s, *cloud_filtered, false);



       
    
	  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud=dest;

  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  // normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  // cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (atof(argv[argc-7]));

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
//  pcl::io::savePolygonFilePLY("mesh.vtk", triangles);
    std::stringstream ss;
    ss << argv[argc-1];
  pcl::io::savePolygonFilePLY(ss.str(), triangles);

    pcl::io::savePLYFile ("stitchTemp.ply", *dest, true);
}
