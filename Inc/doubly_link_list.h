/*
 * doubly_link_list.h
 *
 *  Created on: Sep 27, 2020
 *      Author: vvverma
 */

#ifndef DOUBLY_LINK_LIST_H_
#define DOUBLY_LINK_LIST_H_

typedef struct single_node node;

struct single_node {
	char value;
	node* next;
	node* prev;
};


node* create_list(void);
void del_node(node** addr);
void del_list(node** list);
void add_node(node** list, char val);
void print_list(node* list);
int get_list_length(node** list);



#endif /* DOUBLY_LINK_LIST_H_ */
