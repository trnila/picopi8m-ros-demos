#pragma once
#include <stdarg.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "ros/node_handle.h"
#include "RpmsgHardware.h"


#define ROSLOG_DEFINE(type) \
     void type(const char*fmt, ...) { \
        va_list args; \
        va_start(args, fmt); \
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY); \
        vsnprintf(logbuffer, sizeof(logbuffer), fmt, args); \
        NodeHandle_<T>::type(logbuffer); \
        xSemaphoreGiveRecursive(mutex); \
        va_end(args); \
     }


namespace ros {
  static char logbuffer[128];

  template<typename T>
  class FreeRTOSNodeHandle_ : public NodeHandle_<T> {
    public:
      FreeRTOSNodeHandle_<T>(): NodeHandle_<T>() {
        mutex = xSemaphoreCreateRecursiveMutex();
        configASSERT(mutex);
      }

      virtual int publish(int id, const Msg * msg) override {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        int ret = NodeHandle_<T>::publish(id, msg);
        xSemaphoreGiveRecursive(mutex);
        return ret;
      }

      bool advertise(Publisher & p) {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        bool ret = NodeHandle_<T>::advertise(p);
        xSemaphoreGiveRecursive(mutex);
        return ret;
      }

      template<typename SubscriberT>
      bool subscribe(SubscriberT& s) {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        bool ret = NodeHandle_<T>::subscribe(s);
        xSemaphoreGiveRecursive(mutex);
        return ret;
      }

      template<typename MReq, typename MRes, typename ObjT>
      bool advertiseService(ServiceServer<MReq, MRes, ObjT>& srv) {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        bool ret = NodeHandle_<T>::advertiseService(srv);
        xSemaphoreGiveRecursive(mutex);
        return ret;
      }

      template<typename MReq, typename MRes>
      bool serviceClient(ServiceClient<MReq, MRes>& srv) {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        bool ret = NodeHandle_<T>::serviceClient(srv);
        xSemaphoreGiveRecursive(mutex);
        return ret;
      }

      void negotiateTopics() {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        NodeHandle_<T>::negotiateTopics();
        xSemaphoreGiveRecursive(mutex);
      }

      template<typename ParamType>
      bool getParam(const char* name, ParamType* param, int length = 1, int timeout = 1000) {
        // TODO: params NOT supported YET
        logerror("params not supported yet");
        return false;
      }

      ROSLOG_DEFINE(logdebug)
      ROSLOG_DEFINE(loginfo)
      ROSLOG_DEFINE(logwarn)
      ROSLOG_DEFINE(logerror)
      ROSLOG_DEFINE(logfatal)

    private:
      SemaphoreHandle_t mutex;
  };
  typedef FreeRTOSNodeHandle_<RpmsgHardware> NodeHandle;
}
