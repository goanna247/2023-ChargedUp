#pragma once

#include <networktables/NetworkTable.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

#include <functional>
#include <iostream>

namespace wom {
  class NTBound {
   public:
    NTBound(std::shared_ptr<nt::NetworkTable> table, std::string name, const nt::Value &value, std::function<void(const nt::Value &)> onUpdateFn)
      : _table(table), _entry(table->GetEntry(name)), _onUpdate(onUpdateFn), _name(name) {
        _entry.SetValue(value);
        // _listener = table->AddListener(name, , ([=](const nt::EntryNotification &evt) {
        //   this->_onUpdate(evt.value);
        // }, NT_NOTIFY_UPDATE);
        _listener = table->AddListener(name, nt::EventFlags::kValueAll, ([this](nt::NetworkTable *table, std::string_view key, const nt::Event &event) {
          std::cout << "NT UPDATE" << std::endl;
          this->_onUpdate(event.GetValueEventData()->value);
        }));
      }
    
    NTBound(const NTBound &other) 
      : _table(other._table), _entry(other._entry), _onUpdate(other._onUpdate), _name(other._name) {
      
      _listener = _table->AddListener(_name, nt::EventFlags::kValueAll, ([this](nt::NetworkTable *table, std::string_view key, const nt::Event &event) {
        std::cout << "NT UPDATE" << std::endl;
        this->_onUpdate(event.GetValueEventData()->value);
      }));
    }
    
    ~NTBound() {
      _table->RemoveListener(_listener);
    }
   protected:
    NT_Listener _listener;
    std::shared_ptr<nt::NetworkTable> _table;
    nt::NetworkTableEntry _entry;
    std::function<void(const nt::Value &)> _onUpdate;
    std::string _name;
  };

  class NTBoundDouble : public NTBound {
   public:
    NTBoundDouble(std::shared_ptr<nt::NetworkTable> table, std::string name, double &val)
      : NTBound(table, name, nt::Value::MakeDouble(val), [&val](const nt::Value &v) { val = v.GetDouble(); }) {}
  };

  template <typename T>
  class NTBoundUnit : public NTBound {
   public:
    NTBoundUnit(std::shared_ptr<nt::NetworkTable> table, std::string name, units::unit_t<T> &val)
      : NTBound(table, name, nt::Value::MakeDouble(val.value()), [&val](const nt::Value &v) { val = units::unit_t<T> { v.GetDouble() }; }) {}
  };

  void WritePose2NT(std::shared_ptr<nt::NetworkTable> table, frc::Pose2d pose);
  void WritePose2NT(std::shared_ptr<nt::NetworkTable> table, frc::Pose3d pose);

}