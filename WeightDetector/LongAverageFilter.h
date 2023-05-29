#ifndef LONG_AVERAGE_FILTER_H
#define LONG_AVERAGE_FILTER_H
#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif
extern int cp_num;

class LongAverageFilter {
private:
    struct ListNode {
        int val;
        ListNode* next;

        ListNode() : ListNode(0) {}
        ListNode(int val) : ListNode(val, nullptr) {}
        ListNode(int val, ListNode* next) {
            this->val = val;
            this->next = next;
        }
    };

    ListNode* startLink;
    mydata length;
    mydata average;

    void deleteAll(ListNode* link);

    void push(mydata data);

    void pop();

    void calculateAverageOfLinkedList();

public:
    LongAverageFilter(mydata length);

    void reset();

    void update(mydata data);

    mydata getAverage();

    ~LongAverageFilter();
};

#endif  // LONG_AVERAGE_FILTER_H
