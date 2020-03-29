#pragma once
#include <list>
#include <vector>
using namespace std;

#ifndef BINNODE_H_MY
#define BINNODE_H_MY

template <typename T>
class BinNode {
public:
    T data;
    BinNode<T>* parent;
    BinNode<T>* lChild;
    BinNode<T>* rChild;

    BinNode()
        : parent(NULL), lChild(NULL), rChild(NULL) {}
    BinNode(T e, BinNode<T>* p = NULL, BinNode<T>* lc = NULL, BinNode<T>* rc = NULL)
        : data(e), parent(p), lChild(lc), rChild(rc) {}

    BinNode<T>* insertASLC(T const&);//作为左孩子插入
    BinNode<T>* insertASRC(T const&);//作为右孩子插入

    bool operator==(BinNode const& bn) { return data == bn.data; }
    bool operator==(BinNode<T>* bn) { return data == bn->data; }
    bool operator<(BinNode const& bn) { return data < bn.data; }
    bool operator<(BinNode<T>* bn) { return data < bn->data; }
};

template <typename T>
BinNode<T>* BinNode<T>::insertASLC(T const& e) {
    return lChild = new BinNode(e, this);
}

template <typename T>
BinNode<T>* BinNode<T>::insertASRC(T const& e) {
    return rChild = new BinNode(e, this);
}
#endif // BINNODE_H_MY

#ifndef BINTREE_H_MY
#define BINTREE_H_MY

template <typename T>
class BinTree {
protected:
    int _size;  // total node number
    BinNode<T>* _root;
    vector<BinNode<T>*> _leaves;
public:
    BinTree() :_root(NULL) {}
    ~BinTree() { if (0 < _size) remove(_root); }
    int size() const { return _size; }//用const说明是只读函数
    int depth(BinNode<T>*);
    bool empty() const { return !_root; }
    BinNode<T>* root() const { return _root; }
    void collectLeaves(BinNode<T>* t);
    vector<int> countNumPerLayer(); // 0 locates root
    int countLayerNum(BinNode<T>* , int , int , int );  // 
    // input layer number, returns all nodes in this layer
    vector<BinNode<T>*> getLayerNode(BinNode<T>* , int , int , vector<BinNode<T>*> );
    vector<BinNode<T>*> leaves() { collectLeaves(_root); return _leaves; };
    BinNode<T>* insertASRoot(T const&);
    BinNode<T>* insertASLC(BinNode<T>*, T const&);  // insert as left child
    BinNode<T>* insertASRC(BinNode<T>*, T const&);  // insert as right child
    BinNode<T>* findBro(BinNode<T>*);   //find the brother
    int remove(BinNode<T>*);
    void display(BinNode<T>*);
    void DisplayTree();
    bool operator==(BinTree<T>& other) { return _root && other._root && (_root == other._root); }
};

template <typename T>
int BinTree<T>::depth(BinNode<T>* pRoot) {
    if (pRoot == nullptr)
        return 0;
    int left = depth(pRoot->lChild);
    int right = depth(pRoot->rChild);
    return (left > right) ? (left + 1) : (right + 1);
 }

template <typename T>
vector<int> BinTree<T>::countNumPerLayer() {
    vector<int> NumPerLayer;
    for (int i = 0; i < depth(_root); i++) {
        int current_layer_num = 0;
        int number = 0;
        int depth = 0;
        current_layer_num = countLayerNum(_root, depth, i, number);
        NumPerLayer.push_back(current_layer_num);
    }
    return NumPerLayer;
}

template <typename T>
int BinTree<T>::countLayerNum(BinNode<T>* node, int depth, int k, int number) {
    if (node == NULL)
        return number;
    if (depth == k) {
        number++;
    }
    number = countLayerNum(node->lChild, depth + 1, k, number);
    number = countLayerNum(node->rChild, depth + 1, k, number);
    return number;
}

template <typename T>
vector<BinNode<T>*> BinTree<T>::getLayerNode(BinNode<T>* node, int depth, int k, vector<BinNode<T>*> nodeVec) {
    if (node == NULL)
        return nodeVec;
    if (depth == k) {
        nodeVec.push_back(node);
    }
    nodeVec = getLayerNode(node->lChild, depth + 1, k, nodeVec);
    nodeVec = getLayerNode(node->rChild, depth + 1, k, nodeVec);
    return nodeVec;
}

template <typename T>
void BinTree<T>::collectLeaves(BinNode<T>* t) {
    if (t == NULL) return;
    else {
        collectLeaves(t->lChild);
        if (!t->lChild && !t->rChild)
            _leaves.push_back(t);
        collectLeaves(t->rChild);
    }
}

template <typename T>
BinNode<T>* BinTree<T>::insertASRoot(T const& e) {
    _size = 1;
    _root = new BinNode<T>(e);
    return _root;
}

template <typename T>
BinNode<T>* BinTree<T>::insertASLC(BinNode<T>* t, T const& e) {
    ++_size;
    t->lChild = new BinNode<T>(e);
    if (!t->rChild)
        t->rChild = NULL;
    t->lChild->parent = t;
    return t->lChild;
}

template <typename T>
BinNode<T>* BinTree<T>::insertASRC(BinNode<T>* t, T const& e) {
    ++_size;
    t->rChild = new BinNode<T>(e);
    if (!t->lChild)
        t->lChild = NULL;
    t->rChild->parent = t;
    return t->rChild;
}

template <typename T>
int BinTree<T>::remove(BinNode<T>* t) {
    return removeAt(t);
}

template <typename T>
static int removeAt(BinNode<T>* t) {
    if (!t) return 0;
    int n = 1 + removeAt(t->lChild) + removeAt(t->rChild);
    delete t;
    return n;
}

// find t's brother
template <typename T>
BinNode<T>* BinTree<T>::findBro(BinNode<T>* t) {
    if (!_root || t == NULL)
    {
        printf("Tree or node is empty !\n");
        return _root;
    }
    if (t->parent == NULL) {
        printf("Node is the root, no brother!\n");
        return _root;    // t is root now, no brother
    }
    else {
        if (t->parent->lChild == t)
            return t->parent->rChild;
        else
            return t->parent->lChild;
    }
}

template <typename T>
void BinTree<T>::display(BinNode<T>* t) {
    if (t == NULL) return;
    else {
        std::cout << "[";
        display(t->lChild);
        std::cout << "] " << t->data << " [";
        display(t->rChild);
        std::cout << "]";
    }
}

// 展示二叉树
class DisplayInfo {
public:
    int level;
    int pos;        //结点在屏幕中的绝对位置
    bool enter;
    int spaceNum;
};

template <typename T>
void BinTree<T>::DisplayTree() {
    int i;
    list<BinNode<T>*> Q;
    list<DisplayInfo> QI;
    int screenWidth = 64;
    int dataWidth = 2;
    DisplayInfo info;    //将插入队列的结点的打印信息
    DisplayInfo preInfo; //队尾的结点的打印信息
    BinNode<T>* curNode;       //队列当前取出的结点
    DisplayInfo curInfo; //队列当前取出的结点的打印信息
    if (!_root)
    {
        printf("Tree is empty !\n");
        return;
    }
    printf("Nature Display Tree:\n");
    Q.push_back(_root);
    info.level = 1;
    info.enter = true;
    info.spaceNum = screenWidth >> info.level;
    info.pos = info.spaceNum;
    QI.push_back(info);
    preInfo = info;
    while (Q.size())
    {
        curNode = Q.front();
        Q.pop_front();
        curInfo = QI.front();
        if (curInfo.enter)
            printf("\n\n");
        for (i = 0; i < curInfo.spaceNum; i++)
            printf(" ");
        //printf("%2d", curNode->data);
        for (const int& k : curNode->data)
            cout << k << ",";
        QI.pop_front();
        if (curNode->lChild)
        {
            Q.push_back(curNode->lChild);
            info.level = curInfo.level + 1;
            info.pos = curInfo.pos - (screenWidth >> info.level);
            if (info.level > preInfo.level)
            {
                info.enter = true;
                info.spaceNum = info.pos;
            }
            else
            {
                info.enter = false;
                info.spaceNum = info.pos - preInfo.pos;
            }
            info.spaceNum -= dataWidth;
            QI.push_back(info);
            preInfo = info;

        }
        if (curNode->rChild)
        {
            Q.push_back(curNode->rChild);
            info.level = curInfo.level + 1;
            info.pos = curInfo.pos + (screenWidth >> info.level);
            if (info.level > preInfo.level)
            {
                info.enter = true;
                info.spaceNum = info.pos;
            }
            else
            {
                info.enter = false;
                info.spaceNum = info.pos - preInfo.pos;
            }
            info.spaceNum -= dataWidth;
            QI.push_back(info);
            preInfo = info;
        }
    }
    printf("\n\n\n");
}
#endif // BINNODE_H_MY