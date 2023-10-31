/*
 * ring_buffer.c
 *
 *  Created on: Sep 8, 2023
 *      Author: Vishal Verma
 */

#include<stdio.h>
#include<stdlib.h>


struct ring_buf {
	void *buffer;
	void *head;
	void *tail;

	void *start;
	void *end;
	int size;
	int item_sz;
};

#define _COPY(to, from, size) \
		{ while(size--)\
	*to++=*from++;\
		}

#define _MOVE(ptr, size)\
		{ while(size--)\
	ptr++;\
		}


void ring_buffer_init(struct ring_buf *buf,int item_sz, int count) {
	int size = item_sz*count;
	char *ptr;

	buf->buffer = malloc(size);
	buf->start  =  buf->buffer;

	buf->head = buf->buffer;
	buf->tail = buf->buffer;

	buf->size = size;
	buf->item_sz = item_sz;
	ptr = (char*) buf->start;
	int i = 0;
	while(i < size){
		ptr++;
		i++;
	}

	buf->end = ptr;
}

void ring_buffer_insert(struct ring_buf *buf, const void *item, int item_sz) {


	if (buf->head == buf->end) {
		if(buf->start == buf->tail){
			char* mvtail  = (char*) buf->tail;
			int i = 0;
			while(i < item_sz) {
				mvtail++;
				i++;
			}
		    buf->tail = (void*) mvtail;
		}
		buf->head = buf->start;
		printf("Buffer Overflow\n");
	}


	if((buf->head-buf->tail) == (-1*item_sz)) {
		printf("Buffer is full Detected, Overwriting \n");
		_COPY((char*)buf->head, (char*)item, item_sz);
		item_sz = buf->item_sz;
		char* move = (char*)buf->tail;
		_MOVE(move,item_sz);
		buf->tail = (void*)move;
		if(buf->tail== buf->end)
			buf->tail = buf->start;
	}
	else {
		_COPY((char*)buf->head, (char*)item, item_sz);
	}

}

int ring_buffer_remove(struct ring_buf *buf) {

	if((buf->head == buf->tail)||(buf->head ==buf->end&&buf->tail==buf->start)||(buf->head==buf->end&&buf->tail==buf->start))  {
		printf("Error: Cannot Remove Data, Buffer is empty\n");
		return -1;
	}

	char* ptr = (char*) buf->tail;
	int size = buf->item_sz;
	_MOVE(ptr,size);
	buf->tail = (void*)ptr;

	if(buf->tail == buf->end) {
		buf->tail = buf->start;
	}
	return 0;
}

void* ring_buffer_peek(struct ring_buf *buf) {
	if((buf->head == buf->tail)||(buf->head ==buf->end&&buf->tail==buf->start)||(buf->head==buf->end&&buf->tail==buf->start)) {
		printf("Error: No  Data present, Buffer is empty\n");
		return buf->head;
	}

	return buf->tail;
}

int ring_buffer_delete(struct ring_buf *buf) {

	if(buf == NULL) {
		return -1;
	}
	if(buf->buffer)
		free(buf->buffer);
	return 0;
}

void flush_buffer(struct ring_buf *buf) {
	while(buf->head != buf->tail){
		int *data =  (int *)ring_buffer_peek(buf);
		if(data)
			printf("Dumping Data: %d Adress of head %p Adress of tail %p\n",*data,buf->head,buf->tail);
		ring_buffer_remove(buf);
	}
	int *data =  (int *)buf->head;
	if(data)
		printf("Dumping Data: %d Adress of head %p Adress of tail %p\n",*data,buf->head,buf->tail);

}

/*
int main (){
	struct ring_buf* circular = (struct ring_buf*) malloc(sizeof(struct ring_buf));
	int a = 1;

	ring_buffer_init(circular,sizeof(a),5);
    /*
	for(int i = 0; i < 12 ; i++) {
		ring_buffer_insert(circular,&i,sizeof(int));
	}*/
/*

	for(int i = 0; i < 12 ; i++) {
		ring_buffer_insert(circular,&i,sizeof(int));
		int *data =  (int *)ring_buffer_peek(circular);
		if(data)
			printf("%d\n",*data);
		ring_buffer_remove(circular);
     }

	flush_buffer(circular);
	ring_buffer_delete(circular);
	free(circular);
	return 0;
}*/



