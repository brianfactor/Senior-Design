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

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/
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

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
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
  for (int i = 1; i < argc-4; i++)
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
    Eigen::Affine3f transform=pcl::getTransformation(27,0,14,0,ang,0);
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
	//Statistical filter
  std::cerr << "PointCloud before filtering: " << dest->width * dest->height 
       << " data points (" << pcl::getFieldsList (*dest) << ").";	
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> st;
  st.setInputCloud (dest);
  st.setMeanK (atoi(argv[argc-4]));
  st.setStddevMulThresh (atof(argv[argc-3]));
  st.filter (*dest);
  std::cerr << "PointCloud after filtering: " << dest->width * dest->height 
       << " data points (" << pcl::getFieldsList (*dest) << ").";	


  //writer.write<pcl::PointXYZRGB> ("inliers_"+s, *cloud_filtered, false);


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
       
    
    std::stringstream ss;
    ss << argv[argc-1];
    pcl::io::savePLYFile (ss.str (), *dest, true);
}
