class Throttle {
   public:
    explicit Throttle(int interval) : interval_(interval), count_(0) {}

    bool shouldLog() { return (count_++ % interval_ == 0); }

   private:
    int interval_;
    int count_;
};