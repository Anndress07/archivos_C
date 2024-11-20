#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct treenode {
    int value;
    struct treenode *left;
    struct treenode *right;
} treenode;

treenode *createnode(int value) {
    treenode *result = malloc(sizeof(treenode));
    if (result != NULL) {
        result->left = NULL;
        result->right = NULL;
        result->value = value;
    }
    return result;
}

void printtabs(int numtabs) {
    for (int i = 0; i < numtabs; i++){
        printf("\t");
    }
}

void printtree_rec(treenode *root, int level) {
    if (root == NULL) {
        printtabs(level);
        printf("---- <empty> \n");
        return;
    }
    printtabs(level);
    printf("value: %d\n", root->value);
    printtabs(level);
    printf("left\n");
    

    printtree_rec(root->left, level+1);
    printtabs(level);
    printf("right\n");

    
    printtree_rec(root->right, level+1);
    printtabs(level);
    printf("done\n");
        
}

void printtree(treenode *root) {
    printtree_rec(root, 0); 
}

bool insertnumber(treenode **rootptr, int value){
     treenode *root = *rootptr;
    if (root == NULL) {
        (*rootptr) = createnode(value);
        return true;
    }
    if (value == root->value){
        // do nada
        return false;
    }
    if (value < root->value) {
        return insertnumber(&(root->left), value);
    } else {
        return insertnumber(&(root->right), value);
    }
}

bool findnumber(treenode *root, int value) {
    if (root == NULL) {
        return false;
    }
    if (root->value == value) {
        return true;
    }
    if (value < root->value) {
        return findnumber(root->left, value);
    } else { return findnumber(root->right, value);}
}


int main(){
    treenode *root = NULL;
    insertnumber(&root, 15);
    insertnumber(&root, 11);
    insertnumber(&root, 24);
    insertnumber(&root, 5);
    insertnumber(&root, 19);
    insertnumber(&root, 16);

    printtree(root);
    printf("%d (%d)\n", 16, findnumber(root,16));
    printf("%d (%d)\n", 15,  findnumber(root,15));
    printf("%d (%d)\n", 5, findnumber(root,5));
    printf("%d (%d)\n", 511, findnumber(root,511));
    
}
