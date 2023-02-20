#ifndef IIT_UR5E_JOINT_DATA_MAP_H_
#define IIT_UR5E_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace ur5e {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[SHOULDER_PAN_JOINT] = rhs[SHOULDER_PAN_JOINT];
    data[SHOULDER_LIFT_JOINT] = rhs[SHOULDER_LIFT_JOINT];
    data[ELBOW_JOINT] = rhs[ELBOW_JOINT];
    data[WRIST_1_JOINT] = rhs[WRIST_1_JOINT];
    data[WRIST_2_JOINT] = rhs[WRIST_2_JOINT];
    data[WRIST_3_JOINT] = rhs[WRIST_3_JOINT];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[SHOULDER_PAN_JOINT] = value;
    data[SHOULDER_LIFT_JOINT] = value;
    data[ELBOW_JOINT] = value;
    data[WRIST_1_JOINT] = value;
    data[WRIST_2_JOINT] = value;
    data[WRIST_3_JOINT] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   shoulder_pan_joint = "
    << map[SHOULDER_PAN_JOINT]
    << "   shoulder_lift_joint = "
    << map[SHOULDER_LIFT_JOINT]
    << "   elbow_joint = "
    << map[ELBOW_JOINT]
    << "   wrist_1_joint = "
    << map[WRIST_1_JOINT]
    << "   wrist_2_joint = "
    << map[WRIST_2_JOINT]
    << "   wrist_3_joint = "
    << map[WRIST_3_JOINT]
    ;
    return out;
}

}
}
#endif
