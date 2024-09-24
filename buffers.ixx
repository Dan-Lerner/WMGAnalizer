module;
#include<iterator>
export module buffers;

//import std.core;

namespace buffers
{
    template<typename T, size_t size> class ring_iterator;

    export
    template<typename T, size_t init_buffer_size>
    class ring_buffer final
    {
    public:
        using iterator = ring_iterator<T, init_buffer_size>;
        using const_iterator = ring_iterator<const T, init_buffer_size>;
        using reverse_iterator = std::reverse_iterator<iterator>;
        using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    public:
        int push(const T& v) noexcept 
        {
            buffer_[head_] = v;
            return move_pointers();
        }
        
        int push(T&& v) noexcept 
        {
            buffer_[head_] = std::move(v);
            return move_pointers();
        }
        
        const T& get_at(int index) const noexcept 
        { 
            return buffer_[index]; 
        }
        
        size_t count() const noexcept
        {
            return overhead_ ? init_buffer_size : head_;
        }
        
        size_t difference(int index_last, int index_first) const noexcept
        {
            auto diff = index_last - index_first;
            return diff > 0 ? diff : size_ + diff;
        }
        
        int tail() const noexcept
        {
            return overhead_ ? next(head_) : 0;
        }
        
        int head() const noexcept
        {
            return head_;
        }
        
        int last() const noexcept
        {
            return overhead_ || head_ > 0 ? prev(head_) : -1;
        }
        
        bool overheaded() const noexcept
        {
            return overhead_;
        }
        
        void clear() noexcept 
        {
            head_ = 0;
            overhead_ = false;
        }
        
        int next(int index) const noexcept
        {
            return (index + 1) == size_ ? 0 : index + 1;
        }
        
        int prev(int index) const noexcept
        {
            return (index - 1) < 0 ? init_buffer_size : index - 1;
        }

    public:
        iterator begin() noexcept
        {
            return it(tail());
        }
        
        iterator end() noexcept
        {
            return it(head());
        }
        
        iterator it(int index) noexcept
        {
            return iterator(*this, index);
        }
        
        const_iterator begin() const noexcept
        {
            return const_iterator(buffer_, tail());
        }
        
        const_iterator end() const noexcept
        {
            return const_iterator(buffer_, head());
        }
        
        reverse_iterator rbegin() 
        {
            return std::make_reverse_iterator(this->end());
        }
        
        reverse_iterator rend() 
        {
            return std::make_reverse_iterator(this->begin());
        }
        
        reverse_iterator r_it(int index)  
        {
            return std::make_reverse_iterator(this->it(index));
        }

    public:
        T& operator[](int index) noexcept 
        {
            return buffer_[index];
        }

    private:
        int move_pointers() noexcept 
        {
            auto ret = head_;
            head_ = next(head_);
            if (!overhead_ && head_ == 0)
                overhead_ = true;
            return ret;
        }

    private:
        const size_t size_{ init_buffer_size + 1 };
        T buffer_[init_buffer_size + 1];
        int head_{};
        bool overhead_{};
    };

    export
    template<typename T, size_t size>
    class ring_iterator final
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = std::remove_cv_t<T>;
        using pointer = T*;
        using reference = T&;
        using iterator_category = std::random_access_iterator_tag; //// !!!!!!!!

    public:
        ring_iterator() = default;
        explicit ring_iterator(ring_buffer<T, size>& b, int i) noexcept
            : pbuffer_{ &b }
            , index_{ i }
        {}
        
        ring_iterator(const ring_iterator& it) noexcept
            : pbuffer_{ it.pbuffer_ }
            , index_{ it.index_ }
        {}

    public:
        bool operator!=(ring_iterator const& other) const noexcept
        {
            return index_ != other.index_;
        }
        
        bool operator!=(ring_iterator const& other) noexcept 
        {
            return index_ != other.index_;
        }
        
        bool operator==(ring_iterator const& other) const noexcept
        {
            return index_ == other.index_;
        }
        
        typename ring_iterator::reference operator*() /*const*/ noexcept
        {
            return (*pbuffer_)[index_];
        }
        
        ring_iterator& operator++() noexcept
        {
            index_ = pbuffer_->next(index_);
            return *this;
        }
        
        ring_iterator operator++(int) noexcept 
        {
            ring_iterator copy(*this);
            ++(*this);
            return copy;
        }
        
        ring_iterator& operator--() noexcept 
        {
            index_ = pbuffer_->prev(index_);
            return *this;
        }
        
        ring_iterator operator--(int) noexcept 
        {
            ring_iterator copy(*this);
            --(*this);
            return copy;
        }
        
        int operator-(const ring_iterator& other) noexcept 
        {
            return pbuffer_->difference(index_, other.index_);
        }
        
        ring_iterator& operator=(ring_iterator& other) noexcept
        {
            if (&other != this) {
                pbuffer_ = other.pbuffer_;
                index_ = other.index_;
            }
            return *this;
        }
        
        ring_iterator& operator=(ring_iterator&& other) noexcept
        {
            return operator=(other);
        }

        int index()
        {
            return index_;
        }

    private:
        ring_buffer<T, size>* pbuffer_;
        int index_;
    };
}