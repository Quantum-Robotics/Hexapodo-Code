#include "RFController.h"

RFControl QUR001Control;

void setup()
{
	QUR001Control.Start();
}

void loop()
{
	QUR001Control.Routine();
}