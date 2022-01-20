#include "filter_line/vincent_soille_watershed.h"

vincent_soille_watershed::vincent_soille_watershed()
{
}

vincent_soille_watershed::~vincent_soille_watershed()
{
}

void vincent_soille_watershed::populate_graph_from_array(  const cppimage &i_image, graph_type  &o_graph)
{

  if(o_graph.empty())
    {
      for(size_t kk=0; kk < i_image.get_nbrow(); ++kk)
	{
	  for(size_t ll=0; ll < i_image.get_nbcol(); ++ll)
	    {
	      pixel_type pixel_location (kk,ll);
	      std::pair< int, pixel_type > map_elt ( i_image.get_kl_value(kk,ll), pixel_location);
	      o_graph.insert(map_elt);
	    }
        }
    }
  else
    {
      std::cout<<"WARNING: multimap is not empty!"<<std::endl;
      std::cout<<"         it will be cleaned before populating"<<std::endl;
      o_graph.clear();
      this->populate_graph_from_array(i_image,o_graph);
    }
}



void vincent_soille_watershed::display_graph(const std::multimap< int, std::pair<size_t,size_t> > &i_graph) const
{
  graph_type::const_iterator it;

  for (it=i_graph.begin(); it != i_graph.end(); ++it)
    {
      //std::cout << (*it).first << " => (" << (*it).second.first<< "," << (*it).second.second  << ") \n";
    }
}


//void vincent_soille_watershed::populate_array_from_graph()
//{
//}

std::vector< pixel_type > vincent_soille_watershed::get_neighbors_list(const cppimage &input_image, pixel_type input_pixel, int neighborhood_size )
{
  std::vector< pixel_type > neighbors_list;
  size_t nb_row = input_image.get_nbrow();
  size_t nb_col = input_image.get_nbcol();
  size_t idx_row = input_pixel.first;
  size_t idx_col = input_pixel.second;
  size_t new_idx_row = 0, new_idx_col=0;

  //neighbors_list.push_back( pixel_type(input_pixel.first,input_pixel.second)) ;

  if(neighborhood_size == 4)
    {
      //left
      new_idx_col = idx_col - 1;
      new_idx_row =  idx_row;
      if(idx_col > 0)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}
      //right
      new_idx_col = idx_col + 1;
      new_idx_row =  idx_row;
      if(new_idx_col < nb_col)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}
      //top
      new_idx_row =  idx_row + 1;
      new_idx_col = idx_col;
      if(new_idx_row < nb_row)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}
      //bottom
      new_idx_row =  idx_row - 1;
      new_idx_col = idx_col;
      if( idx_row  > 0)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}

    }
  else if(neighborhood_size == 8)
    {

      //left
      new_idx_col = idx_col - 1;
      new_idx_row =  idx_row;
      if( idx_col > 0)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}
      //right
      new_idx_col = idx_col + 1;
      new_idx_row =  idx_row;
      if(new_idx_col < nb_col)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}
      //top
      new_idx_row =  idx_row - 1;
      new_idx_col = idx_col;
      if( idx_row > 0)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}
      //bottom
      new_idx_row =  idx_row + 1;
      new_idx_col = idx_col;
      if(new_idx_row  < nb_row)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}

      // top left
      new_idx_col = idx_col - 1;
      new_idx_row =  idx_row - 1;
      if(new_idx_col >= 0 && idx_row > 0)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}


      // top right
      new_idx_col = idx_col + 1;
      new_idx_row =  idx_row - 1;
      if(new_idx_col < nb_col && idx_row > 0)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}


      // bottom left
      new_idx_col = idx_col - 1;
      new_idx_row =  idx_row + 1;
      if( idx_col > 0 && new_idx_row < nb_row)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}

      // bottom right
      new_idx_col = idx_col + 1;
      new_idx_row =  idx_row + 1;
      if(new_idx_col < nb_col && new_idx_row < nb_row)
	{
	  neighbors_list.push_back(pixel_type (new_idx_row, new_idx_col));
	}

    }
  else
    {
      std::cout<<"BAOWW !! MAUVAIS VOISINAGE " << std::endl;
    }

  return neighbors_list; // vecteur de max 8 elt on s'autorise la copie
}

void vincent_soille_watershed::get_labelled_array(double *lab_out_for_matlab) const
{
  const std::vector<int> &lab_array = lab_w.get_image_array();

  size_t cpt = 0;

  std::vector<int>::const_iterator hmax = std::max_element(lab_array.begin(), lab_array.end());
  std::vector<int>::const_iterator hmin = std::min_element(lab_array.begin(), lab_array.end());

  for(std::vector<int>::const_iterator it=lab_array.begin(); it!=lab_array.end(); ++it)
    {
      //std::cout<<"copie pour "<< cpt;
      lab_out_for_matlab[cpt] = static_cast<double>(*it);
      //std::cout<<" valide "<<std::endl;
      cpt++;
    }



}


void vincent_soille_watershed::process_watershed_algo(const cppimage &input_im, int connectivity)
{  //  vincent soille watershed algo

  int init_tag = -1;
  int mask_tag = -2;
  int wshed_tag = 0;
  pixel_type fictitious = pixel_type(-1,-1);

  int curlab  = 0;
  assert( this->fifo.empty()); // ligne 9 algo

  lab_w = cppimage(input_im.get_nbrow(), input_im.get_nbcol(), init_tag); //ligne 10 to 12
  cppimage dist = cppimage(input_im.get_nbrow(), input_im.get_nbcol(), 0);  // algo


  const std::vector< int > &image_array = input_im.get_image_array();

  this->populate_graph_from_array(input_im, this->image_graph); // ligne 13 algo
  //this->display_graph(this->image_graph);


  std::vector<int>::const_iterator hmax = std::max_element(image_array.begin(), image_array.end());
  std::vector<int>::const_iterator hmin = std::min_element(image_array.begin(), image_array.end());
  std::cout<< "hmin = " << *hmin << " hmax = " << *hmax << std::endl;

  // DEBUG SANITY CHECK
  std::multimap< int ,std::pair<size_t,size_t> >::iterator it;
  it = this->image_graph.begin();
  assert( (*hmin) == (*it).first);

  it = --(this->image_graph.end());
  assert( (*hmax) == (*it).first);
  //END DEBUG SANITY CHECK


  for (int level=(*hmin); level<=(*hmax); ++level)
    {

      std::pair< graph_type::iterator , graph_type::iterator >  pixels_at_level_it;
      pixels_at_level_it = this->image_graph.equal_range(level);

      //std::cout<<" ----- start_flooding -----"  << std::endl;
      //std::cout<<"|- processing level "<< level << std::endl;


     for( graph_type::iterator map_it = pixels_at_level_it.first; map_it !=  pixels_at_level_it.second; ++map_it)
	{

	  //std::cout<< "(" << (*map_it).second.first <<", "<< (*map_it).second.second << ") ; " ;
	  lab_w.set_kl_value((*map_it).second.first, (*map_it).second.second, mask_tag); // lab[p] = mask (ligne 18)
	  std::vector< pixel_type > neighbors_list = this->get_neighbors_list(input_im,  (*map_it).second, connectivity);
	  for(std::vector< pixel_type >::iterator it=neighbors_list.begin(); it != neighbors_list.end(); ++it)
	    {
	      pixel_type current_neighbor = *it; // q dans l'algo
	      if(lab_w.get_kl_value(current_neighbor) > 0 || lab_w.get_kl_value( current_neighbor) == wshed_tag)
		{
		  dist.set_kl_value( (*map_it).second, 1);
		  this->fifo.push( (*map_it).second);
		  break;
		}
	    }
	}
     int curdist = 1;
     this->fifo.push(fictitious);

     while(true)
       {
	 pixel_type current_pixel = this->fifo.front(); // p dans l'algo (ligne27)
	 this->fifo.pop();
	 if( current_pixel.first == fictitious.first &&  current_pixel.second == fictitious.second)
	   {
	     if(this->fifo.empty())
	       {
		 break;
	       }
	     else
	       {
		 this->fifo.push(fictitious);
		 curdist += 1;
		 current_pixel = this->fifo.front();
		 this->fifo.pop();
	       }
	   } // endif (ligne 35 algo)

	 std::vector< pixel_type > cur_neighbors_list = this->get_neighbors_list(input_im,  current_pixel, connectivity);

	 for(std::vector< pixel_type >::iterator it=cur_neighbors_list.begin(); it != cur_neighbors_list.end(); ++it)
	   {
	     pixel_type current_neighbor = *it;// q ligne 36 dans l'algo
	     if( (dist.get_kl_value( current_neighbor) < curdist )  && ( lab_w.get_kl_value( current_neighbor) > 0 || lab_w.get_kl_value( current_neighbor ) == wshed_tag ) )
	       {
		 if( lab_w.get_kl_value( current_neighbor) > 0)  // ligne 39
		   {
		     if(lab_w.get_kl_value(current_pixel) == mask_tag || lab_w.get_kl_value(current_pixel) == wshed_tag ) // ligne 40
		       {
			 lab_w.set_kl_value(current_pixel,lab_w.get_kl_value( current_neighbor) );
		       }
		     else if( lab_w.get_kl_value(current_pixel) != lab_w.get_kl_value( current_neighbor ) )// lig90 42
		       {
			 lab_w.set_kl_value(current_pixel,wshed_tag);
		       }
		   }
		 else if (lab_w.get_kl_value(current_pixel) == mask_tag) // ligne 45
		   {
		     lab_w.set_kl_value(current_pixel,wshed_tag);
		   }// enfdif (ligne 47 algo)
	       }
	     else if(lab_w.get_kl_value( current_neighbor)  == mask_tag && dist.get_kl_value(current_neighbor) ==0) //ligne 48
	       {
		 dist.set_kl_value( current_neighbor, curdist + 1);
		 this->fifo.push(current_neighbor);
	       }// endif (ligne 50 algo)
	   } //endfor (ligne 51 algo)

       }//end while

     // detect and process new minima at level h
     for( graph_type::iterator map_it = pixels_at_level_it.first; map_it !=  pixels_at_level_it.second; ++map_it)
       {
	 pixel_type current_pixel = (*map_it).second;
	 dist.set_kl_value( current_pixel, 0);
	 if(lab_w.get_kl_value( current_pixel) == mask_tag)
	   {
	     curlab += 1;
	     this->fifo.push(current_pixel);
	     lab_w.set_kl_value(current_pixel, curlab);
	     while( !this->fifo.empty())
	       {
		 pixel_type removed_pix = this->fifo.front();
		 this->fifo.pop();
		 std::vector< pixel_type > cur_neighbors_list = this->get_neighbors_list(input_im, removed_pix, connectivity);
		 for(std::vector< pixel_type >::iterator it=cur_neighbors_list.begin(); it != cur_neighbors_list.end(); ++it)
		   {
		     pixel_type current_neighbor = *it; // r ligne 61
		     if (lab_w.get_kl_value(current_neighbor) == mask_tag)
		       {
			 this->fifo.push( current_neighbor);
			 lab_w.set_kl_value(current_neighbor, curlab);
		       }
		   }

	       }
	   } // endif (ligne 67 algo)
       } // endfor (ligne 68 algo)


     //std::cout<< std::endl;
     //lab_w.display_image_tab();
     //std::cout<< std::endl << "|-  level "<< level << " complete" <<  std::endl;
    } // end loop on level

  //loop to mark all watershed point
  for ( graph_type::iterator it = this->image_graph.begin(); it!= this->image_graph.end(); ++it)
    {
      pixel_type current_pixel = (*it).second;
      std::vector< pixel_type > neighbors_list = get_neighbors_list(input_im, current_pixel, 4 );
      int current_label = lab_w.get_kl_value(current_pixel);
      for (std::vector<pixel_type>::iterator nit=neighbors_list.begin(); nit!=neighbors_list.end(); ++nit)
	{
	  pixel_type  current_neighbor = *nit;

	  if(lab_w.get_kl_value(current_neighbor) != wshed_tag && lab_w.get_kl_value(current_neighbor)<current_label)
	    {
	      lab_w.set_kl_value(current_pixel, wshed_tag);
	      break;
	    }
	}

    }

  //lab_w.display_image_tab();
} // end function


