/******************************************************************************
 * @file    Mean_filter.h
 * @authors  LH
 * @date    2016/09/12
 * @brief   Mean filtering class
 ******************************************************************************/
#include <assert.h>
#include <queue>
#include <vector>

#ifndef HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_MEAN_FILTER_H_
#define HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_MEAN_FILTER_H_

template<typename TT>
  struct mean_filter_struct
  {

    std::queue<std::vector<TT> > datas;

    std::vector<TT> sum;

  };

template<class T>
  class mean_filter
  {
  public:

    mean_filter()
    {
      _window = 1;
    }
    ;

    void set_window(int size)
    {
      assert(size > 0);
      _window = size;
    }
    ;

    void add_data(std::vector<T> data)
    {

      if (_datas.datas.size() == 0)
      {
        _datas.datas.push(data);

        for (int I = 0; I < data.size(); I++)
        {
          _datas.sum.push_back(data.at(I));
        }
      }

      else if (_datas.datas.size() == _window)
      {
        std::vector<T> old_data = _datas.datas.front();

        for (int I = 0; I < old_data.size(); I++)
        {
          _datas.sum.at(I) = _datas.sum.at(I) - old_data.at(I) + data.at(I);
        }
        _datas.datas.pop();
        _datas.datas.push(data);
      }

      else
      {
        for (int I = 0; I < data.size(); I++)
        {
          _datas.sum.at(I) += data.at(I);
        }
        _datas.datas.push(data);
      }

    }
    ;

    void run_filter(std::vector<T> &result)
    {
      assert(_datas.datas.size() > 0);
      if(result.size() != _datas.datas.front().size()){
        result.resize(_datas.datas.front().size(),T(0.0));

      }

      for (int I = 0; I < _datas.datas.front().size(); I++)
      {
        result.at(I) = _datas.sum.at(I) / _datas.datas.size();
      }

    }
    ;

  private:

    mean_filter_struct<T> _datas;

    int _window;

  };

#endif /* HUMANOID_NAVIGATION_CONTROLLER_FORCE_OBSERVER_INCLUDE_MEAN_FILTER_H_ */
