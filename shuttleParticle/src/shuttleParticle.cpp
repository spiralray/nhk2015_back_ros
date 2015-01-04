#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <math.h>
#include <opencv2/legacy/legacy.hpp>

// (1)尤度の計算を行なう関数
float
calc_likelihood (IplImage * img, int x, int y)
{
  float b, g, r;
  float dist = 0.0, sigma = 50.0;

  b = img->imageData[img->widthStep * y + x * 3];       //B
  g = img->imageData[img->widthStep * y + x * 3 + 1];   //G
  r = img->imageData[img->widthStep * y + x * 3 + 2];   //R
  dist = sqrt (b * b + g * g + (255.0 - r) * (255.0 - r));

  return 1.0 / (sqrt (2.0 * CV_PI) * sigma) * expf (-dist * dist / (2.0 * sigma * sigma));
}


int
main (int argc, char **argv)
{
  int i, c;
  double w = 0.0, h = 0.0;
  CvCapture *capture = 0;
  IplImage *frame = 0;

  int n_stat = 6;
  int n_particle = 4000;
  CvConDensation *cond = 0;
  CvMat *lowerBound = 0;
  CvMat *upperBound = 0;

  int xx, yy;

  // (2)コマンド引数によって指定された番号のカメラに対するキャプチャ構造体を作成する
  if (argc == 1 || (argc == 2 && strlen (argv[1]) == 1 && isdigit (argv[1][0])))
    capture = cvCreateCameraCapture (argc == 2 ? argv[1][0] - '0' : 0);

  // (3)１フレームキャプチャし，キャプチャサイズを取得する．
  frame = cvQueryFrame (capture);
  w = frame->width;
  h = frame->height;

  cvNamedWindow ("Condensation", CV_WINDOW_AUTOSIZE);

  // (4)Condensation構造体を作成する．
  cond = cvCreateConDensation (n_stat, 0, n_particle);

  // (5)状態ベクトル各次元の取りうる最小値・最大値を指定する．
  lowerBound = cvCreateMat (n_stat, 1, CV_32FC1);
  upperBound = cvCreateMat (n_stat, 1, CV_32FC1);

  cvmSet (lowerBound, 0, 0, 0.0);
  cvmSet (lowerBound, 1, 0, 0.0);
  cvmSet (lowerBound, 2, 0, -10.0);
  cvmSet (lowerBound, 3, 0, -10.0);
  cvmSet (lowerBound, 4, 0, -10.0);
  cvmSet (lowerBound, 5, 0, -10.0);

  cvmSet (upperBound, 0, 0, w);
  cvmSet (upperBound, 1, 0, h);
  cvmSet (upperBound, 2, 0, 10.0);
  cvmSet (upperBound, 3, 0, 10.0);
  cvmSet (upperBound, 4, 0, 10.0);
  cvmSet (upperBound, 5, 0, 10.0);

  // (6)Condensation構造体を初期化する
  cvConDensInitSampleSet (cond, lowerBound, upperBound);

  // (7)ConDensationアルゴリズムにおける状態ベクトルのダイナミクスを指定する
  cond->DynamMatr[0] = 1.0;
  cond->DynamMatr[1] = 0.0;
  cond->DynamMatr[2] = 1.0;
  cond->DynamMatr[3] = 0.0;
  cond->DynamMatr[4] = 0.0;
  cond->DynamMatr[5] = 0.0;

  cond->DynamMatr[6] = 0.0;
  cond->DynamMatr[7] = 1.0;
  cond->DynamMatr[8] = 0.0;
  cond->DynamMatr[9] = 1.0;
  cond->DynamMatr[10] = 0.0;
  cond->DynamMatr[11] = 0.0;

  cond->DynamMatr[12] = 0.0;
  cond->DynamMatr[13] = 0.0;
  cond->DynamMatr[14] = 1.0;
  cond->DynamMatr[15] = 0.0;
  cond->DynamMatr[16] = 0.0;
  cond->DynamMatr[17] = 0.0;

  cond->DynamMatr[18] = 0.0;
  cond->DynamMatr[19] = 0.0;
  cond->DynamMatr[20] = 0.0;
  cond->DynamMatr[21] = 1.0;
  cond->DynamMatr[22] = 0.0;
  cond->DynamMatr[23] = 0.0;

  cond->DynamMatr[24] = 0.0;
  cond->DynamMatr[25] = 0.0;
  cond->DynamMatr[26] = 0.0;
  cond->DynamMatr[27] = 0.0;
  cond->DynamMatr[28] = 0.0;
  cond->DynamMatr[29] = 0.0;

  cond->DynamMatr[30] = 0.0;
  cond->DynamMatr[31] = 0.0;
  cond->DynamMatr[32] = 0.0;
  cond->DynamMatr[33] = 0.0;
  cond->DynamMatr[34] = 0.0;
  cond->DynamMatr[35] = 0.0;

  // (8)ノイズパラメータを再設定する．
  cvRandInit (&(cond->RandS[0]), -25, 25, (int) cvGetTickCount ());
  cvRandInit (&(cond->RandS[1]), -25, 25, (int) cvGetTickCount ());
  cvRandInit (&(cond->RandS[2]), -5, 5, (int) cvGetTickCount ());
  cvRandInit (&(cond->RandS[3]), -5, 5, (int) cvGetTickCount ());
  cvRandInit (&(cond->RandS[4]), -5, 5, (int) cvGetTickCount ());
  cvRandInit (&(cond->RandS[5]), -5, 5, (int) cvGetTickCount ());

  while (1) {
    frame = cvQueryFrame (capture);

    // (9)各パーティクルについて尤度を計算する．
    for (i = 0; i < n_particle; i++) {
      xx = (int) (cond->flSamples[i][0]);
      yy = (int) (cond->flSamples[i][1]);
      if (xx < 0 || xx >= w || yy < 0 || yy >= h) {
        cond->flConfidence[i] = 0.0;
      }
      else {
        cond->flConfidence[i] = calc_likelihood (frame, xx, yy);
        cvCircle (frame, cvPoint (xx, yy), 2, CV_RGB (0, 0, 255), -1);
      }
    }

    cvShowImage ("Condensation", frame);
    c = cvWaitKey (10);
    if (c == '\x1b')
      break;

    // (10)次のモデルの状態を推定する
    cvConDensUpdateByTime (cond);

  }

  cvDestroyWindow ("Condensation");
  cvReleaseImage (&frame);
  cvReleaseCapture (&capture);
  cvReleaseConDensation (&cond);
  cvReleaseMat (&lowerBound);
  cvReleaseMat (&upperBound);

  return 0;
}
