/*
 * Interface to gsl_point_id.c, Silead's touch-point tracking (GNU GPL version 2
 * or later; see that file, LICENSE-GPL-2.0.txt and the README's License section).
 */
#ifndef _GSL_POINT_ID_H
#define _GSL_POINT_ID_H

struct gsl_touch_info
{
    int x[10];
    int y[10];
    int id[10];
    int finger_num;
};

unsigned int gsl_mask_tiaoping(void);
unsigned int gsl_version_id(void);
void gsl_alg_id_main(struct gsl_touch_info *cinfo);
void gsl_DataInit(unsigned int *conf_in);

#endif

