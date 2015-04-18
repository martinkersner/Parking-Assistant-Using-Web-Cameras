#ifndef PROPERTIES_H_
#define PROPERTIES_H_

#define IMG_WIDTH       640
#define IMG_HEIGHT      480
#define CAM_FPS         30

#define CHESS_ROWS      9
#define CHESS_COLS      6 
#define SQUARE_SIZE     2.8
#define CALIB_TIMESPAN  100
#define CALIB_IMAGES    21
#define CALIB_DIR       "Chessboard/"

#define LEFT_CAM_IDX    1
#define RIGHT_CAM_IDX   2
#define CAM_PROP_INT    "intrinsics.yml"
#define CAM_PROP_EXT    "extrinsics.yml"
#define CAM_PROP_DIS    "distortion_camera_models.yml"

#endif // PROPERTIES_H_
