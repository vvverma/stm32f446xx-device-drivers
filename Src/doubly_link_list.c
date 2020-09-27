/*
 * doubly_link_list.c
 *
 *  Created on: Sep 27, 2020
 *      Author: vvverma
 */


#include <doubly_link_list.h>
#include <stdlib.h>

node* create_list(void){
	node* list;
	list =(node*)malloc(sizeof(node));
	list->next = NULL;
	list->prev = NULL;
	list->value = '\0';
	return list;
}
void del_node(node** addr){
	node* saveAddr;
	saveAddr = *addr;
	if (saveAddr == NULL)
		return;

	//head node only
	if (saveAddr->next == NULL && saveAddr->prev == NULL)
		return;


	// last node
	if(saveAddr->next == NULL){
		saveAddr->prev->next = NULL;
	}//first node
	else if(saveAddr->prev == NULL){
		saveAddr->next->prev = NULL;
	}
	else{
		saveAddr->prev->next = saveAddr->next;
		saveAddr->next->prev = saveAddr->prev;
	}

	free(saveAddr);
}

void del_list(node** list) {
	node* temp;
	node* saveAddr;
	temp = *list;

	while(temp!=NULL){
		saveAddr = temp->next;
		free(temp);
		temp = saveAddr;
	}
	free(*list);
}
void add_node(node** list, char val){
	node* temp, *newnode;

	newnode = (node*) malloc(sizeof(node));
	newnode->next = NULL;
	newnode->prev = NULL;
	newnode->value = val;

	temp = *list;
	if(temp->next == NULL){
		temp->next = newnode;
		newnode->prev = temp;
	}
	else {

		while(temp->next !=NULL){
			temp = temp->next;
		}
		temp->next = newnode;
		newnode->prev = temp;
	}
}

void print_list(node* list){
	node* temp;
	temp = list->next;
	while(temp!=NULL){
		if (temp-> value != '\0'){
		//putchars(temp->value);
		temp = temp->next;
		}
	}
}

int get_list_length(node** list){
	node* temp;
	int count;
	count =0;
	temp = *list;
	while(temp != NULL){
		count++;
		temp = temp->next;
	}
	return count-1;
}
