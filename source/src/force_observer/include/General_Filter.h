/******************************************************************************
 * @file    General_Filter.h
 * @authors  LH
 * @date    2016/09/13
 * @brief   A general filter class
 *          This class implements a discrete-time filter. The filter design must
 *          be done beforehand. This class takes the filter numerator and
 *          denominator as an input and can then be used to filter datas.
 ******************************************************************************/
#include<vector>
#ifndef HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_BUTTERWORTH_FILTER_H_
#define HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_BUTTERWORTH_FILTER_H_
template<class T>
  class lowpass_filter
  {
  public:

    lowpass_filter()
    {
      _past_input = T(0.0);
      _past_output = T(0.0);
    }
    ;

    /*******************************************************************************
     * @brief Main class function. Run the filter with the input data
     * @param input [in] The input data
     * @return The output of the filter
     ******************************************************************************/
    T run(T input)
    {

      T output;
      output = (_numerator.at(0) * input + _numerator.at(1) * _past_input
          - _denominator.at(1) * _past_output) / _denominator.at(0);
      _past_input = input;
      _past_output = output;
      return output;
    }
    ;

    /*******************************************************************************
     * @brief Set the filter coefficients
     * @param numerator [in] Vector containing the filter numerator
     * @param denominator [in] Vector containing the filter denominator
     ******************************************************************************/
    void set_coefficient(std::vector<T> numerator, std::vector<T> denominator)
    {

      for (int I = 0; I < numerator.size(); I++)
      {

        _numerator.push_back(numerator.at(I));
      }

      for (int I = 0; I < denominator.size(); I++)
      {

        _denominator.push_back(denominator.at(I));
      }

    }

  private:

    std::vector<T> _numerator;
    std::vector<T> _denominator;

    T _past_input;
    T _past_output;

  };

#endif /* HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_BUTTERWORTH_FILTER_H_ */
