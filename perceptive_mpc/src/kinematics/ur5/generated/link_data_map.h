#ifndef IIT_UR5E_LINK_DATA_MAP_H_
#define IIT_UR5E_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace ur5e {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE_LINK_UR5E] = rhs[BASE_LINK_UR5E];
    data[SHOULDER_LINK] = rhs[SHOULDER_LINK];
    data[UPPER_ARM_LINK] = rhs[UPPER_ARM_LINK];
    data[FOREARM_LINK] = rhs[FOREARM_LINK];
    data[WRIST_1_LINK] = rhs[WRIST_1_LINK];
    data[WRIST_2_LINK] = rhs[WRIST_2_LINK];
    data[WRIST_3_LINK] = rhs[WRIST_3_LINK];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE_LINK_UR5E] = value;
    data[SHOULDER_LINK] = value;
    data[UPPER_ARM_LINK] = value;
    data[FOREARM_LINK] = value;
    data[WRIST_1_LINK] = value;
    data[WRIST_2_LINK] = value;
    data[WRIST_3_LINK] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base_link_ur5e = "
    << map[BASE_LINK_UR5E]
    << "   shoulder_link = "
    << map[SHOULDER_LINK]
    << "   upper_arm_link = "
    << map[UPPER_ARM_LINK]
    << "   forearm_link = "
    << map[FOREARM_LINK]
    << "   wrist_1_link = "
    << map[WRIST_1_LINK]
    << "   wrist_2_link = "
    << map[WRIST_2_LINK]
    << "   wrist_3_link = "
    << map[WRIST_3_LINK]
    ;
    return out;
}

}
}
#endif
