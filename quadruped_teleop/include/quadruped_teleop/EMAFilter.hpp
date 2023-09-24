#ifndef EMA_FILTER_HPP
#define EMA_FILTER_HPP

#include <cmath>

class EMAFilter
{
public:
  EMAFilter();
  EMAFilter(int filter_size);
  EMAFilter(double beta);
  ~EMAFilter();

  void resetFilter();

  double filterData(double raw_data);
  void setStartVal(double data);

  int getFilterSize();
  void setFilterSize(int filter_size);

  double getFilterBeta();
  void setFilterBeta(double beta);

private:
  int _num_observations;
  int _filter_size;
  double _beta;

  double _filtered_data;
  double _previous_data_filtered;

};

EMAFilter::EMAFilter()
{
  this->resetFilter();
}

EMAFilter::EMAFilter(int filter_size) : _filter_size(filter_size)
{
  this->_beta = 1.0 - 1.0/filter_size;
  this->resetFilter();
}

EMAFilter::EMAFilter(double beta) : _beta(beta)
{
  this->_filter_size = (int) (1.0/(1.0-beta));
  this->resetFilter();
}

EMAFilter::~EMAFilter()
{
}

void EMAFilter::resetFilter()
{
  this->_previous_data_filtered = 0.0;
  this->_num_observations = 0;
}

void EMAFilter::setStartVal(double data)
{
  this->_previous_data_filtered = data;
  this->_filtered_data = data;
  this->_num_observations = 0;
}

int EMAFilter::getFilterSize()
{
  return this->_filter_size;
}

void EMAFilter::setFilterSize(int filter_size)
{
  this->_filter_size = filter_size;
  this->_beta = 1.0 - 1.0/filter_size;
}

double EMAFilter::getFilterBeta()
{
  return this->_beta;
}

void EMAFilter::setFilterBeta(double beta)
{
  this->_beta = beta;
  this->_filter_size = (int) (1.0/(1.0-beta));
}

double EMAFilter::filterData(double raw_data)
{
  this->_num_observations++;

  this->_filtered_data = _beta*_previous_data_filtered + (1.0-_beta)*raw_data;
  this->_previous_data_filtered = _filtered_data;

  if (_num_observations < _filter_size) this->_filtered_data /= (1.0 - pow(_beta, _num_observations));

  return this->_filtered_data;
}

#endif // EMA_FILTER_HPP
