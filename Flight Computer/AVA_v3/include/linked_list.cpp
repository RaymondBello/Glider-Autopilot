#include <Arduino.h>

struct Node {
    Node(int d, Node *n): 
        data(d), next(n) {
    }
    Node *next;
    int data;
};

void TraverseLinkedList(const Node* head) {
    const Node *curHead = head;
    while (curHead) {
        Serial.println(curHead->data);
        curHead = curHead->next;
    }
}

void TestLinkedList() {

    Node *n3 = new Node(4, NULL);
    Node *n2 = new Node(1, n3);
    Node *n1 = new Node(3, n2);

    TraverseLinkedList(n1);

    delete n1;
    delete n2;
    delete n3;
    
}