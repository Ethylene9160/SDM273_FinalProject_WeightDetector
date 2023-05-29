#include "LongAverageFilter.h"

//int cp_num = 0;

LongAverageFilter::LongAverageFilter(mydata length) {
    average = 0;
    if (length < 2) length = 2;
    this->length = length;

    ListNode* node = new ListNode(0);
    startLink = new ListNode(0, node);
    cp_num += 2;
    for (int i = 2; i < length; ++i) {
        node->next = new ListNode(0);
        node = node->next;
        cp_num++;
    }
}

void LongAverageFilter::deleteAll(ListNode* link) {
    while (link != nullptr) {
        ListNode* l = link;
        link = l->next;
        delete l;
        cp_num--;
    }
}

void LongAverageFilter::push(mydata data) {
    ListNode* node = startLink;
    while (node->next != nullptr) {
        node = node->next;
    }
    node->next = new ListNode(data);
    cp_num++;
}

void LongAverageFilter::pop() {
    ListNode* node = startLink;
    startLink = startLink->next;
    delete node;
    cp_num--;
}

void LongAverageFilter::reset() {
    ListNode* node = startLink;
    while (node != nullptr) {
        node->val = 0;
        node = node->next;
    }
    average = 0;
}

void LongAverageFilter::calculateAverageOfLinkedList() {
    mydata sum = 0;
    int len = 0;
    ListNode* node = startLink;
    while (node != nullptr) {
        sum += node->val;
        len++;
        node = node->next;
    }
    average = sum / len;
}

void LongAverageFilter::update(mydata data) {
    pop();
    push(data);
    calculateAverageOfLinkedList();
}

mydata LongAverageFilter::getAverage() {
    return average;
}

LongAverageFilter::~LongAverageFilter() {
    cp_num--;
    deleteAll(startLink);
}
