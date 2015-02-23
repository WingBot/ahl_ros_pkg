#ifndef __TRAIN_WITH_CG_TRAIN_HPP
#define __TRAIN_WITH_CG_TRAIN_HPP

#include <boost/shared_ptr.hpp>
#include <neural_network/neural_network.hpp>

namespace train
{

  class Train
  {
  public:
    Train();

    void start();
    void save();
  private:
    nn::NeuralNetworkPtr nn_;

  };

  typedef boost::shared_ptr<Train> TrainPtr;
}

#endif /* __TRAIN_WITH_CG_TRAIN_HPP */
