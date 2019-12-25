/*
    高效循环队列 模板类
	Author: He Guojian (writen in 2018-2019)
	与c++ stl中queue区别：事先申请好固定大小的连续线性空间，循环队列机制；
									满了之后新的覆盖旧的；
									使用非常灵活，内存效率高。
	注意：队列的front是有值的，rear指向的那个位置是不存在的。
*/

#ifndef HQUE_H
#define HQUE_H

#include <vector>
#define DEFAULT_QUEUE_CAPACITY 16

template<class T>
class HQue
{
public:
	HQue();									//不指定capacity，默认DEFAULT_QUEUE_CAPACITY
	HQue(const int _capacity);			//指定最大capacity
	~HQue();

	void push(std::vector<T>&);		//从尾部添加一组元素
	void push(const T&);  				//从尾部添加一个元素
	void pop_front();  						//从头部删掉一个元素
	void pop_back();  						//从尾部删掉一个元素

	void clear();								//清空所有元素
	int size();									//返回元素个数
	void reserve(const int _len);		//重新调整队列capacity大小
	int capacity();  							//获取capacity

	//下面函数都涉及到index，index无直观意义，
	//只能配合这几个函数使用
	T& at(const int index);
	T& operator[](const int i);
	int front();							//获取队列头虚拟index
	int rear();								//获取队列尾虚拟index
	int get_next(const int i);		//获取i的下一个元素
	int get_before(const int i);		//获取i的上一个元素

public:
	static HQue<T>* GetInstance();  //单例模式

private:
	bool Init();

private:
	int m_len;
	int m_rear;
	int m_front;
	T* m_base;
	bool is_success_init;//标志是否成功进行初始化
	int m_space_size; //内存中实际的容量长度

private:
	static HQue<T>* pDG;  //单例模式
};




template<class T>
HQue<T>* HQue<T>::pDG = 0;

template<class T>
HQue<T>::HQue()
{
	//注意，内存中实际队列长度是capacity+1，为rear留一个空
	m_space_size = DEFAULT_QUEUE_CAPACITY + 1; 
    is_success_init = Init();
}

template<class T>
HQue<T>::HQue(const int _capacity)
{
	//注意，内存中实际队列长度是capacity+1，为rear留一个空
	m_space_size = _capacity + 1;  //给rear多留一个空间
	is_success_init = Init();
}

template<class T>
HQue<T>::~HQue()
{
    if(m_base)
        delete[] m_base;
    m_base = NULL;
    m_len = 0;
	m_front =	m_rear = 0;
}

template<class T>
HQue<T> *HQue<T>::GetInstance()
{
    if(pDG == 0)
        pDG = new HQue;
    return pDG;
}

template<class T>
bool HQue<T>::Init()
{
    m_base = new T[m_space_size]; 
    if(!m_base)
		return false;
	m_rear = m_len = m_front = 0;
	return true;
}

template<class T>
void HQue<T>::reserve(const int _len)
{
    is_success_init = false;
    if(m_base)
        delete []m_base;
	
	m_space_size = _len + 1;
    m_base = new T[m_space_size];
    if(m_base)
        is_success_init = true;
    m_rear = m_len = m_front = 0;
}

template<class T>
void HQue<T>::push(std::vector<T> &contents)
{
    if(false == is_success_init)
        return;
    for(int i = 0; i < contents.size(); ++i)
    {
        m_base[m_rear] = contents[i];
		++m_rear;
        m_rear = m_rear % m_space_size;
		++m_len;
		
		//新的覆盖旧的，front被挤着移动1位
		if(m_rear == m_front)  
		{
			++m_front;
			m_front = m_front % m_space_size;
			--m_len;
		}
    }
}

template<class T>
void HQue<T>::push(const T &t)
{
    if(false == is_success_init)
        return;
   
    m_base[m_rear] = t;
	++m_rear;
    m_rear = m_rear % m_space_size;
	++m_len;
	
	//新的覆盖旧的，front被挤着移动1位
	if(m_rear == m_front)
	{
		++m_front;
		m_front = m_front % m_space_size;
		--m_len;
	}
}

template<class T>
void HQue<T>::pop_front()
{
	if(m_len)
	{
		++m_front;
		m_front = m_front % m_space_size;
		--m_len;
	}
}

template<class T>
void HQue<T>::pop_back()
{
	if(m_len)
	{
		if(m_rear==0)
			m_rear = m_space_size - 1;
		else --m_rear;
		--m_len;
	}
}

template<class T>
int HQue<T>::size()
{
    return m_len;
}

template<class T>
int HQue<T>::capacity()
{
    return m_space_size - 1;
}

template<class T>
void HQue<T>::clear()
{
     m_len = 0;
	 m_rear = m_front = 0;
}

//===============================================
//以下函数请注意，都涉及到了整型值下标 index，
//但这个index没有直观意义，
//对index加减操作不会得到想要的结果，必须结合get_next、get_before等函数使用

template<class T>
int HQue<T>::front()
{
	return m_front;
}

template<class T>
int HQue<T>::rear()
{
	return m_rear;
}

template<class T>
T& HQue<T>::at(const int index)
{
    return m_base[index];
}

template<class T>
T& HQue<T>::operator[](const int i)
{
    return m_base[i];
}

template<class T>
int HQue<T>::get_next(const int i)
{
	return ((i+1) % m_space_size);
}

template<class T>
int HQue<T>::get_before(const int i)
{
	if(i==0)
		return m_space_size - 1;
	else 
		return i-1;
}


#endif // HQUE_H
