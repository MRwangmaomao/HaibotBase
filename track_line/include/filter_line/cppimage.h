#ifndef ___CPPIMAGE___
#define ___CPPIMAGE___

/*!
 * \file cppimage.h
 * \brief a simple image container (image is stored as a vector)
 * \author jwojak
 */

#include <vector>
#include <iostream>
#include <assert.h>
#include <algorithm>
#include <map>

/*! 
 *  \brief to descripe a pixel as pair of coordinate (first row, second column)
 */
typedef std::pair< size_t, size_t > pixel_type;

/*! \class cppimage
 *  simple image container class
 */
class cppimage
{
 public:
  /*!
   * \brief Constructor
   *  default constructor 
   */
  cppimage();

  /*!
   * \brief Constructor
   * constructor of a constant image for a given size 
   *
   * \param nbrow: number of rows
   * \param nbcol: number of colums
   * \param init_val: constant value to initialize pixel value of the image
   */
  cppimage(size_t nbrow, size_t nbcol, int init_val);

  /*!
   * \brief Constructor
   * constructor dedicated to matlba interface where the input image array 
   * is given throw a pointer on double
   *
   * \param nbrow: number of rows
   * \param nbcol: number of colums
   * \param init_array: pointer of an array from mex file (warning! despite the double* type 
   *                    the input array is supposed to be quantified into integer value a static_cast
   *                    is done here
   */
  cppimage(size_t nbrow, size_t nbcol,  double *init_array);

  /*!
    \brief Destructor
    default destrutor
   */
  ~cppimage();

  /*!
   * \brief number of row getter
   */
  size_t get_nbrow() const;

  /*!
   * \brief number of column getter
   */
  size_t get_nbcol() const;
  
  /*!
   * \brief number of pixel getter
   */
  size_t get_nbelt() const;
  
  /*!
   * \brief number of row setter
   * \param nbrow: number of row
   */
  void set_nbrow(size_t nbrow);

  /*!
   * \brief number of column setter
   * \param nbcol: number of column
   */
  void set_nbcol(size_t nbcol);

  /*!
   * \brief get the value of the image at the pixel located at given coordinate
   * \param idx_row: row coordinate of pixel 
   * \param idx_col: col coordinate of pixel
   */
  int get_kl_value(size_t idx_row, size_t idx_col) const;
  
  /*!
   * \brief get the value of the image at the pixel located at given coordinate
   * \param pixel: coordinates of the pixel
   */
  int get_kl_value(pixel_type pixel) const;

  /*!
   * \brief set the value of the image at the pixel located at given coordinate
   * \param idx_row: row coordinate of pixel 
   * \param idx_col: col coordinate of pixel
   * \param value: value of the pixel
   */
  void set_kl_value(size_t idx_row, size_t idx_col, int value);
  
  /*!
   * \brief set the value of the image at the pixel located at given coordinate
   * \param pixel: coordinates of the pixel
   * \param value: value of the pixel
   */
  void set_kl_value(pixel_type pixel, int value);

  /*!
   * \brief get the min intensity value of an image
   */
  int get_im_min() const;
  
  /*!
   * \brief get the max intensity value of an image
   */
  int get_im_max() const;
  
  /*!
   *\brief init an image of previously setted size with the value 0
   */
  void init_empty_image();
  
  /*!
   * \brief display image as tab of value on the standard output
   */
  void display_image_tab() const;

  /*!
   * \brief getter of the  image array (simple vector of int)
   */							\
  const std::vector<int> & get_image_array() const;
  
  
 private:
  size_t nb_row; /*!< number of rows*/
  size_t nb_col; /*!< number of colums*/
  size_t nb_elt; /*!< number of pixels in the image*/
  std::vector< int > image_array; /* image array of int (assuming the image is quantified using integer) */

  void set_nbelt();/*!< nb_row*nb_col*/
 
};

#endif
