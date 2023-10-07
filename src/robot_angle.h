#ifndef M5STACK_ROBOTANGLE_H
#define M5STACK_ROBOTANGLE_H

struct RobotAngle{
  union
  {
    float value[3];
    struct
    {
      float x;
      float y;
      float z;
    };
  };
};

#endif //M5STACK_ROBOTANGLE_H
