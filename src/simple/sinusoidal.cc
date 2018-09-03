#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/common/proto/call_python.h"

class Sinusoid : public drake::systems::LeafSystem<double>
{
public:
    Sinusoid (double tstart, double freq, double amp, double offset) :
        m_freq(freq), m_amp(amp), m_offset(offset), m_tstart(tstart) {
    this->DeclareVectorOutputPort(
            drake::systems::BasicVector<double>(2), &Sinusoid::output);
        }

private:
    void output(const drake::systems::Context<double>& c,             drake::systems::BasicVector<double>* output) const {
      double t(c.get_time());
      double tknot(t - m_tstart);
      std::cout << "t = " << t << std::endl;
      if (t > m_tstart) {
          output->SetAtIndex(0, std::sin(tknot*m_freq +     m_offset)*m_amp);
          output->SetAtIndex(1, std::cos(tknot*m_freq +     m_offset)*m_amp*m_freq);
      } else {
          output->SetAtIndex(0, 0.0);
          output->SetAtIndex(1, 0.0);
      }
    }
    double m_freq{0.0}, m_amp{0.0}, m_offset{0.0}, m_tstart{0.0};
};

int main(int argc, char *argv[])
{
    // Add System and Connect
    drake::systems::DiagramBuilder<double> builder;
    auto system  = builder.AddSystem<Sinusoid>(1.0, 2.*M_PI*1., 3., 0.);
    auto logger = LogOutput(system->get_output_port(0), &builder);
    logger->
    auto diagram = builder.Build();

    // Construct Simulator
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_publish_every_time_step(true);
    simulator.get_mutable_integrator()->set_fixed_step_mode(true);
    simulator.get_mutable_integrator()->set_maximum_step_size(0.1);

    // Run simulation
    simulator.StepTo(100);

    // Plot with Python
    auto sample_time = logger->sample_times();
    auto sample_data = logger->data();
    std::cout << sample_time.size() << std::endl;
    for (int i = 0; i < sample_time.size(); ++i) {
        std::cout << sample_time(i) << " : " << sample_data(i, 0) << "      " << sample_data(i, 1) << std::endl;
        }

    std::cout << "END" << std::endl;
    return 0;
}
