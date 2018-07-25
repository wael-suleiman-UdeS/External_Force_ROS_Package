/******************************************************************************
 * @file    Schmitt_trigger.h
 * @authors  LH
 * @date    2016/09/12
 * @brief   Schmitt trigger implementation
 ******************************************************************************/
#ifndef HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_SCHMITT_TRIGGER_H_
#define HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_SCHMITT_TRIGGER_H_

template<class T>
  class schmitt_trigger
  {
  public:

    schmitt_trigger():_state(false){};

    void set_threshold(T Pos_Threshold, T Neg_Threshold)
    {
      _Positive_Threshold = Pos_Threshold;
      _Negative_Threshold = Neg_Threshold;
    }
    ;

    bool run(T signal)
    {

      if (signal >= _Positive_Threshold && !_state)
      {
        _state = true;
      }
      else if (signal >= _Negative_Threshold && _state)
      {
        _state = true;
      }
      else
        _state = false;

      return _state;
    }
    ;

  private:

    bool _state;
    T _Positive_Threshold;
    T _Negative_Threshold;

  };

#endif /* HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_SCHMITT_TRIGGER_H_ */
