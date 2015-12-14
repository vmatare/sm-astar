/*
 * load_png.h
 *
 *  Created on: 25.10.2015
 *      Author: ich
 */

#ifndef LOAD_PNG_H_
#define LOAD_PNG_H_

#include <png.h>


class PNGReaderData
{
 public:
  FILE *infile;
  png_structp png_ptr;
  png_infop info_ptr;
  int number_passes;
  bool read;
};



std::vector<unsigned char> loadPNG(std::string filename) {

	std::vector<unsigned char> rv;

	PNGReaderData *d = new PNGReaderData();
	d->read = false;

	if ((d->infile = fopen(filename.c_str(), "rb")) == NULL) {
		throw std::exception();
	}

	d->png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

	if (d->png_ptr == NULL) {
		fclose(d->infile);
		throw std::exception();
	}

	d->info_ptr = png_create_info_struct(d->png_ptr);
	if (d->info_ptr == NULL) {
		fclose(d->infile);
		png_destroy_read_struct(&d->png_ptr, (png_infopp)NULL, (png_infopp)NULL);
		throw std::exception();
	}

	if (setjmp(png_jmpbuf(d->png_ptr))) {
		/* Free all of the memory associated with the png_ptr and info_ptr */
		png_destroy_read_struct(&d->png_ptr, &d->info_ptr, (png_infopp)NULL);
		fclose(d->infile);
		/* If we get here, we had a problem reading the file */
		throw std::exception();
	}

	png_init_io(d->png_ptr, d->infile);
	png_read_info(d->png_ptr, d->info_ptr);
	png_set_strip_16(d->png_ptr);
	png_set_strip_alpha(d->png_ptr);
	png_set_packing(d->png_ptr);

	png_byte color_type = png_get_color_type(d->png_ptr, d->info_ptr);
	if (color_type == PNG_COLOR_TYPE_PALETTE)  png_set_palette_to_rgb(d->png_ptr);
	if (color_type == PNG_COLOR_TYPE_GRAY)     png_set_gray_to_rgb(d->png_ptr);

	int intent;
	double screen_gamma = 2.2;  /* A good guess for a PC monitors in a dimly lit room */
	if (png_get_sRGB(d->png_ptr, d->info_ptr, &intent)) {
		png_set_gamma(d->png_ptr, screen_gamma, 0.45455);
	} else {
		double image_gamma;
		if (png_get_gAMA(d->png_ptr, d->info_ptr, &image_gamma)) {
			png_set_gamma(d->png_ptr, screen_gamma, image_gamma);
		} else {
			png_set_gamma(d->png_ptr, screen_gamma, 0.45455);
		}
	}

	d->number_passes = png_set_interlace_handling(d->png_ptr);
	png_read_update_info(d->png_ptr, d->info_ptr);

	png_bytep row_pointer;
	row_pointer = (png_bytep)png_malloc(d->png_ptr, png_get_rowbytes(d->png_ptr, d->info_ptr));

	unsigned int lheight = png_get_image_height(d->png_ptr, d->info_ptr);
	unsigned int lwidth  = png_get_image_width(d->png_ptr, d->info_ptr);

	for (int pass = 0; pass < d->number_passes; ++pass) {
		for (unsigned y = 0; y < lheight; ++y) {
			png_read_rows(d->png_ptr, &row_pointer, (png_bytepp)NULL, 1);
		}
	}

	/* read rest of file, and get additional chunks in info_ptr - REQUIRED */
	png_read_end(d->png_ptr, d->info_ptr);
	png_free(d->png_ptr, row_pointer);

	return rv;

}







#endif /* LOAD_PNG_H_ */
