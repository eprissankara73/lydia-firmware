/* 
 * File:   epaper.h
 * Author: Viswanath
 *
 * Created on January 21, 2019, 7:12 AM
 */

#ifndef EPAPER_H
#define	EPAPER_H

#include <xc.h> 
#include "epaper_images.h"


#define VERT_LENGTH  3
#define LINE_LENGTH 2
#define THICK_LINE_LENGTH 3
extern int vert_x[3]; // (0) vertex a, (1)vertex b, (2) vertex c vertices of triangle
extern int vert_y[3]; // (0) vertex a, (1)vertex b, (2) vertex c vertices of triangle
extern int b_vert_x[4]; // bounding rectangle x coords for triangle drawn by vertices
extern int b_vert_y[4]; // bounding rectangle y coords for triangle drawn by vertices
extern int b2_vert_x[4]; // bounding rectangle x coords for triangle drawn by vertices; second array
extern int b2_vert_y[4]; // bounding rectangle y coords for triangle drawn by vertices; second array
extern int line_x[2];
extern int line_y[2];
extern int thick_line_x[4];
extern int thick_line_y[4];
extern int u_rect_x[2];
extern int u_rect_y[2];
extern int digit_pos_x[2];
extern int digit_pos_y[2];
#define DIGIT_POS_LENGTH  2
#define D_PT_LENGTH  600
extern int d_pt[600];
extern int d_pt_count;
extern int d_pt2[600];
extern int d_pt2_count;
#define EP_WIDTH  176
#define EP_HEIGHT 264

#ifdef thickdial
#define DIAMETER  202 //174//170
#define TEMP_IND_OFFSET 28
#define TEMP_IND_LENGTH 14
#else
#define DIAMETER  178 //170
#define TEMP_IND_OFFSET 12
#define TEMP_IND_LENGTH 12
#endif

#define EP_BLACK  0x01
#define EP_WHITE  0x00
#define CENTERX   (EP_HEIGHT/2)
#define CENTERY   (EP_WIDTH - 50)

#define HEAT_INDICATOR 1
#define COOL_INDICATOR 2
#define TEMPERATURE_INDICATOR 3


#define UPDATE_SCREEN_BYTE_LEN 75
extern int up_screen_b[75];

extern int action_pos_x;
extern int action_pos_y;
#define DISPLAY_ACTION_NONE  0x02
#define DISPLAY_ACTION_COOL  0x00
#define DISPLAY_ACTION_HEAT  0x01
extern int cool_sp_min_x;
extern int cool_sp_min_y;
extern int cool_sp_max_x;
extern int cool_sp_max_y;
extern int heat_sp_min_x;
extern int heat_sp_min_y;
extern int heat_sp_max_x;
extern int heat_sp_max_y;
extern int temperature_min_x;
extern int temperature_min_y;
extern int temperature_max_x;
extern int temperature_max_y;
//extern int draw_rect_min_x;
//extern int draw_rect_min_y;
//extern int draw_rect_max_x;
//extern int draw_rect_max_y;
extern int timer_icon_x;
extern int timer_icon_y;



#ifdef	__cplusplus
extern "C" {
#endif

void writeCMD(uint8_t command);
void writeData(uint8_t data);
void epaper_init(void);
void epaper_on(void);
void epaper_off(void);
void setRegisterLUT();
void setOTPLUT();
float cp_sign(int p1_x, int p1_y, int p2_x, int p2_y, int p3_x, int p3_y);
int pointInTriangle(int pt_x, int pt_y, int v1_x, int v1_y, int v2_x, int v2_y, int v3_x, int v3_y);
void app_fill_triangle(int t0_x, int t0_y, int t1_x, int t1_y, int t2_x, int t2_y, int append);
void rotate_triangle(int angle_deg, int centerX, int centerY);
void map_triangle_coords();
void map_digit_coords();
void update_indicator(int type_indicator);
void draw_indicator(int centerX, int centerY, int diameter, int angle);
void draw_gauge();
void draw_range_indicators(int heat_setpoint_degF, int cool_setpoint_degF);
int indexInBuffer(int tmp_index, int* loc_d_pt, int loc_d_pt_count);
void drawTemperature(uint8_t current_temp);
void epaper_draw_screen(uint8_t cool_set_point, uint8_t heat_set_point);
void clearTemperatureIndicator(uint8_t prev_current_temp);
void epaper_draw_heatcool(uint8_t heatorcool);
void epaper_draw_battery(uint8_t draw_batt);
void draw_heat_sp_indicator(int heat_setpoint_degF);
void draw_cool_sp_indicator(int cool_setpoint_degF);
void clear_indicator(int min_x, int min_y, int max_x, int max_y);
void epaper_draw_timer(int draw_timer);
void epaper_draw_plusminus(int draw_plusminus);
void epaper_draw_officon(int draw_officon);
void clear_both_indicators(int min_x1, int min_y1, int max_x1, int max_y1,
        int min_x2, int min_y2, int max_x2, int max_y2);
void draw_heatcool_sp_indicators(int heat_setpoint_degF, int cool_setpoint_degF);
void epaper_draw_cwire(uint8_t draw_cwire);

#ifdef	__cplusplus
}
#endif

#endif	/* EPAPER_H */

