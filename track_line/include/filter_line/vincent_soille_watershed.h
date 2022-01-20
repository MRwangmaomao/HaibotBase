#ifndef ___VINCENT_SOILLE_WATERSHED___
#define ___VINCENT_SOILLE_WATERSHED___

/*!
  *\file vincent_soille_watershed.h
  * \brief an implementation of the vincent&soille algorithm proposed in
  * "Watersheds in Digital Spaces: An Efficient Algorithm Based on Immersion Simulations"
  *  (IEEE PAMI 1991)
  * \author jwojak
 */

#include "cppimage.h"
#include <algorithm>
#include <queue> 

/*
 * \brief to describe an image as a sorted list of pixels (increasing intensity order) 
 *        (giving the list of intensity + pixel coordinates pair)
 */
typedef std::multimap< int, pixel_type  > graph_type;

/*! \class vincent_soille_watershed
 *  process the algo on a image of class cppimage
 */
class vincent_soille_watershed
{
 public:
  /*!
   * \brief Constructor
   *  default constructor 
   */
  vincent_soille_watershed();
  
  /*!
   * \brief Destructor
   * default destrutor
   */
  ~vincent_soille_watershed();
  
   /*!
    * \brief process the algo
    * \param input_im: a const ref to the input image 
    * \param connectivity; neighborhood coonectivity, default value is 4
    */
  void process_watershed_algo(const cppimage &input_im, int connectivity = 4);
  
  /*
   * \brief export the resulting whsed labelled tab to matlab
   *        by a pointer of double (mex)
   * \param lab_out_for_matlab: pointer of double which can be get by matlab mex
   */
  void get_labelled_array(double *lab_out_for_matlab) const;

 private:
  cppimage lab_w; /*!< labelled wshed image (the result of the algo) */
  graph_type image_graph; /*!< input image viewed as a list of pixels */
  std::queue< pixel_type > fifo; /*!< queue */
  
  /*
   * \brief from input image fill the ordered pixel list
   * \param input_image: input image (see cppimage.h for details)
   * \param output_graph: ordered list of pixel (intensity increasing order)
   */
  void populate_graph_from_array( const cppimage &input_image, graph_type &output_graph ); 
  
  /*
   * \brief display an ordered list of pixels on standard output
   * \param i_graph: ordered list of pixels
   */
  void display_graph(const graph_type &i_graph) const;

  
  //void populate_array_from_graph();
  
  /*	    
   * \brief get coordinate list of neighbor of a given input pixel coordinate
   * \param input_image: input image (it gives the dimention of the image)
   * \param input_pixel: coordiates of the central pixel
   * \param neighborhood_size: coonectivity for neighborhood  or (7 or 8)
   */
  std::vector< pixel_type > get_neighbors_list(const cppimage &input_image, pixel_type input_pixel, int neighborhood_size );
  
};
#endif
