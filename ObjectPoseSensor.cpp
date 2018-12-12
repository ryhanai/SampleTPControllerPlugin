/**
   @author Ryo Hanai
*/

#include "ObjectPoseSensor.h"

namespace teaching
{

  bool ObjectPoseSensor::recognize (std::vector<CompositeParamType>& params, bool isReal)
  {
    // 第一引数：
    // 　オブジェクトの種類（とりあえず1の場合は適当な値を返し、0の場合は認識失敗としておく。
    // 第二引数：
    // 　結果を出力する変数（out引数）

    int object_id = boost::get<int>(params[0]);
    printLog("recognize(", object_id, ", _out_arg)");

    if (object_id == 1) {
      VectorXd x(6);
      x[0] = 0.8; x[1] = 0.2; x[2] = 0.0;
      x[3] = 0.0; x[4] = 0.0; x[5] = 0.0;
      CompositeParamType retVal = x;
      params[1] = retVal;
      return true;
    } else {
      return false;
    }
  }

  bool ObjectPoseSensor::recognize_double (std::vector<CompositeParamType>& params, bool isReal)
  {
    // 第一引数：
    // 　オブジェクトの種類（とりあえず1の場合は適当な値を返し、0の場合は認識失敗としておく。
    // 第二引数：
    // 　結果を出力する変数（out引数）

    int object_id = boost::get<int>(params[0]);
    printLog("recognize_double(", object_id, ", _out_arg)");

    if (object_id == 1) {
      double x = 3.14;
      CompositeParamType retVal = x;
      params[1] = retVal;
      return true;
    } else {
      return false;
    }
  }

  bool ObjectPoseSensor::recognize_int (std::vector<CompositeParamType>& params, bool isReal)
  {
    // 第一引数：
    // 　オブジェクトの種類（とりあえず1の場合は適当な値を返し、0の場合は認識失敗としておく。
    // 第二引数：
    // 　結果を出力する変数（out引数）

    int object_id = boost::get<int>(params[0]);
    printLog("recognize_double(", object_id, ", _out_arg)");

    if (object_id == 1) {
      double x = 777;
      CompositeParamType retVal = x;
      params[1] = retVal;
      return true;
    } else {
      return false;
    }
  }

}
