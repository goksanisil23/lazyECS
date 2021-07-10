#pragma once

#include <iostream>
#include <typeindex>
#include <unordered_map>
#include <vector>
#include <functional>

#include "Types.hpp"

namespace lazyECS {

// Base class to hold all specializations of MemberFunctionHandler in a container
class FunctionHandlerBase {
private:
    // To be implemented by MemberFunctionHandler
    virtual void call(const Event& event) = 0;

public:
    // Call the member function
    void exec(const Event& event) {
        call(event);
    }
};

// Template class to handle the member function (callback) of a concrete System, for a concrete Event.
// Note that MemberFunctionHandler does not have System as a template, since the constructor takes an std::function object,
// which is already tied to a concrete System's method
// --------------- 1 MemberFunctionHandler per callback --------------- // 
template<class TEventType>
class MemberFunctionHandler : public FunctionHandlerBase {

public:
    // Generic member function object. Takes an Event as input, returns nothing. (Member function to be used as Event callback)
    //  std::function signature:
    //  std::function<return_type(input_param_types) functor_name
    typedef std::function<void(const TEventType&)> MemberFunction;

    // Constructor
    explicit MemberFunctionHandler(MemberFunction memberFunc) 
        : member_func_{memberFunc} {};


    // Calls the member function (callback) from the instance of the System
    void call(const Event& event) override {
        // call the member function, with the concrete Event type that this member function expects
        member_func_(static_cast<const TEventType&>(event));
    }

private:
    MemberFunction member_func_; // member function of a concrete System object, to be used as an event callback  
};


// ---------------------- EventBus ---------------------- // 
// For each type of Event, we will keep a list of member func. handlers, to call the concrete callback functions in the System for that Event type
typedef std::vector<FunctionHandlerBase*> FuncHandlerList;

class EventBus {

public:
    template<class TEventType>
    void subscribe(std::function<void(const TEventType&)> memberFunc) {
        // we take the member (callback) function (that is tied to a concrete System object) for this type of Event as input parameter
        FuncHandlerList* func_handlers_per_event = subscriber_map_[typeid(TEventType)];

        if(func_handlers_per_event == nullptr) {
            // 1st subscriber to this Event type
            func_handlers_per_event = new FuncHandlerList();
            subscriber_map_[typeid(TEventType)] = func_handlers_per_event;
        }

        func_handlers_per_event->push_back(
            new MemberFunctionHandler<TEventType>(memberFunc)
        );
    }

    // execute all callback functions that have subscribed to this type of concrete Event
    template<class TEventType>
    void publish(const TEventType& event) {
        FuncHandlerList* func_handlers_per_event = subscriber_map_[typeid(TEventType)];
        
        if(func_handlers_per_event == nullptr) {
            std::__throw_runtime_error("No subscribers for this type of event");
            return;
        }

        for(auto& func_handler : *func_handlers_per_event) {
            if(func_handler != nullptr) {
                func_handler->exec(event);
            }
            else {
                std::__throw_runtime_error("Tried to access null function handler!");
            }
        }
    }
    
private:
    std::unordered_map<std::type_index, FuncHandlerList*> subscriber_map_; // set of callback functions, per Event type
};

} // namespace lazyECS