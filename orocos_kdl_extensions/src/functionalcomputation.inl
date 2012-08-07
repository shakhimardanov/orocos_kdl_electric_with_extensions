#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP

template <typename T>
Example<T>::Example()
{

}

template <typename T>
void Example<T>::push (T const& elem)
{
    elems.push_back(elem);
}

#endif	/* FUNCTIONALCOMPUTATION_HPP */
