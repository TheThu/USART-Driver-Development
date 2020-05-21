/*
 * main.h
 *
 *  Created on: May 10, 2020
 *      Author: T2
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
	int  Matrikelnummer;  //
	char Name[100];
	char Studienfach[50];
	char Geburtsjahr[15];
	int  Semesterzahl;
}STUDENT_INFO_t;



void delete_record(STUDENT_INFO_t *students);
void add_record(STUDENT_INFO_t *students);
void display_all_records(STUDENT_INFO_t *students,u_int32_t length);



#endif /* MAIN_H_ */
