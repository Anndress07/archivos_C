#include <stdio.h>
#include <stdlib.h>

struct node {
    int value;
    struct node* next;

};

typedef struct node node_t;

void printlist(node_t *head){
    node_t *temporary = head;

    while (temporary != NULL) {
        printf("%d - ", temporary->value);
        temporary = temporary->next;
    }
    printf("\n");
}

node_t *create_new_node(int value) {
    node_t *result = malloc(sizeof(node_t));
    result->value = value;
    result->next = NULL;
    return result;

}

node_t *insert_at_head(node_t **head, node_t *node_to_insert) {
    node_to_insert->next = *head;
    *head = node_to_insert;
    return node_to_insert;
}



void insert_after_node(node_t *node_to_insert_after, node_t *newnode) {
    newnode->next = node_to_insert_after->next;
    // printf("newnode->next = %p\t", newnode->next);
    node_to_insert_after->next = newnode;
   

}

void *find_node(node_t *head, int value) {
    node_t *tmp = head;
    while (tmp != NULL) {
        if (tmp->value == value) return tmp;
        tmp = tmp->next;
    }

}

void remove_node(node_t **head, node_t *node_to_remove) {

    if (*head == node_to_remove) {
        *head = node_to_remove->next;
        return;
    } else {
        node_t* temp = *head;
        while (temp !=NULL && temp->next != node_to_remove) {
            temp = temp->next;
        }
        if (temp == NULL) return;
        temp->next = node_to_remove->next;
        node_to_remove->next = NULL;
    }
    return;
}

int main() {

    node_t *head = create_new_node(1);;
    node_t *current = head; 
    node_t *tmp;

    for (int i = 0; i < 25; i++) {
        tmp = create_new_node(i);
        insert_at_head(&head, tmp);
        // insert_after_node(head, tmp);
        // insert_at_head(&head, tmp);
        
    }
    // tmp = create_new_node(32);
    // head = tmp;
    // tmp = create_new_node(8);
    // tmp->next = head;
    // head = tmp;
    // tmp = create_new_node(34);
    // tmp->next=head;
    // head = tmp;
    tmp = find_node(head, 13);
    insert_after_node(tmp, create_new_node(75));

    

    //head = head->next;

    printlist(head);

    printf("current head - %d\n", *head);

    return 0;
}

