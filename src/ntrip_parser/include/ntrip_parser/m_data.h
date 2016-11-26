// -*- C++ -*-
//
// $Id: m_data.h,v 1.1.1.1 2006/01/11 09:34:31 mervart Exp $
//

#if !defined(__m_data_h__)
// protects the header from been included more than once
#define __m_data_h__

#include <time.h>
#include <ctype.h>

#include <string>
#include <vector>
#include <map>

#include <typeinfo>

#include <cstdarg>
#include <iomanip>


#include <iostream>
#include <fstream>

#if defined(__GNUC_MINOR__) && ( __GNUC_MINOR__ < 91 )
#else
using namespace std;
#endif

template<class T>
void WriteNative(ostream &os,const T & data) {
#if 1
  os.write(reinterpret_cast<const char *>(&data),sizeof(data));
#else
  int i;
  os << typeid(data).name() << " " << sizeof(data) << " ";
  for(i= 0;i<sizeof(data);i++) {
    int byte  (  ((unsigned char *)(&data))[i] );
    os << hex << " " << byte;
  }
  os << endl;
#endif
}

template<class T1,class T2>
void WriteNative(ostream & os,const map<T1,T2> &m) {
  WriteNative(os,m.size());
  typedef typename map<T1,T2>::iterator I;
  I i;
  for(i=m.begin();i<m.end();i++) {
    WriteNative(os,*i);
  }
}


// C_Data<T> Wrapper for Scalar types T

template<class T>
class C_Data {
public:
  typedef T DataType;

  explicit C_Data(DataType _data= 0):data(_data) {}
  C_Data(const C_Data & org):data(org.get()) {}
  C_Data &operator= ( const C_Data &newValue) {
    set(newValue.get());
    return *this;
  }
  // bool operator == (T other) { return get() == other.get(); }
  // bool operator != (T other) { return get() != other.get(); }

  ~C_Data() {}

  void writeNative(ostream &os) const {
    WriteNative(os,data);
  }
  
  void set(const DataType &newValue) {
    data = newValue;
  }

  const DataType &get() const {
    return data;
  }

protected:
  DataType data;
} ;

template <class T>
bool operator<(const C_Data<T> &d1,const C_Data<T> &d2) {
  return d1.get() < d2.get();
}

template <class T>
bool operator>(const C_Data<T> &d1,const C_Data<T> &d2) {
  return d1.get() > d2.get();
}

template <class T>
bool operator==(const C_Data<T> &d1,const C_Data<T> &d2) {
  return d1.get() == d2.get();
}

template <class T>
bool operator!=(const C_Data<T> &d1,const C_Data<T> &d2) {
  return d1.get() != d2.get();
}

template <class T>
bool operator<=(const C_Data<T> &d1,const C_Data<T> &d2) {
  return     ( d1.get() == d2.get() ) 
	      || ( d1.get() < d2.get()  ) ;
}
 
template <class T>
C_Data<T> operator-(const C_Data<T> &d1,const C_Data<T> &d2) {
  return C_Data<T>(d1.get() - d2.get());
}

template <class T>
void WriteNative(ostream &os,const C_Data<T> c_data) {
  c_data.writeNative(os);
}




template <class T2,class T3>
void divmod(const long numer,const long denom,T2 &quot,T3 &rem) {
  ldiv_t res;
  ldiv(numer,denom);
  quot = res.quot;
  rem  = res.rem;
}


#endif
