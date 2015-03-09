#ifndef __TRAIN_WITH_CG_AUTO_ENCODER_HPP
#define __TRAIN_WITH_CG_AUTO_ENCODER_HPP

#include <boost/shared_ptr.hpp>
#include <neural_network/neural_network.hpp>

namespace train
{

  class AutoEncoder
  {
  public:
    AutoEncoder();

    void init();
    void train();

  private:
    nn::NeuralNetworkPtr nn_;
    nn::ConfigPtr config_;
    nn::TrainingDataPtr data_;

    std::string config_full_path_;
    std::string data_in_path_;
    std::string data_name_;
    std::string extension_;
    std::string result_full_path_;

    bool use_image_data_;
  };

  typedef boost::shared_ptr<AutoEncoder> AutoEncoderPtr;
}

#endif /* __TRAIN_WITH_CG_AUTO_ENCODER_HPP */
