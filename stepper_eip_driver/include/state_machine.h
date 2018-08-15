#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

namespace stepper_eip_driver {

//events
struct host_control{};
struct enable{};
struct disable{};
struct find_home{};
struct homed{};
struct moving{};
struct safety_stop{};

//transition actions
void profile_move(moving const&);
void look_for_home_sw(find_home const&);
void host_takes_control(host_control const&);
void enable_on(enable const&);
void enable_off(disable const&);
void e_stop_on(safety_stop const&);

struct motor_ : public msm::front::state_machine_def<motor_>
{
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    //someting todo on startup
  }
  template <class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    //someting todo at the end; probably nothing
  }

  //Motor states start here
  struct Init : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  struct HostControl : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  struct Disabled : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  struct Enabled: public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  struct Homing: public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  struct Homed: public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  struct Moving: public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  struct InPosition: public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const&, FSM&) { }
    template <class Event, class FSM>
    void on_exit(Event const&, FSM&) { }
  };

  //initial statye must be defined
  typedef Init initial_state;

};

typedef motor_ m; //make transition table cleaner

struct transition_table : mpl::vector<
    //      Start   Event     Next      Action      Guard
    //    +-------+--------+--------+-----------+--------------+
  a_row   < Init  , >,
> {};

}
#endif // STATE_MACHINE_H
