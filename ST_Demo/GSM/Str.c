void strncpyExNewLine(char *to, char *from, char size) {
  // Copies characters from one string to another, looking for the
  // end of string or checking for a maximum size (maximum number of
  // characters copied),
  // then appends new line characters to the end
  // (size includes the new line and null-terminating characters)
  char ctr = 0;
  size -= 3;
  while (*from != 0 && ctr < size) {
    *to++ = *from++;
    ctr++;
  }
  *to++ = 13;
  *to++ = 10;
  *to = 0; // Mark end of string
}

char strcpyTillChar(char *source, char *dest, char stop_char, char max_len) {
  // Copies from one string to another until a specific character is found
  // (the stop character is not included / copied)
  // Returns the number of characters copied
  char ctr = 0;
  while (*source != stop_char && ctr < max_len) {
    *dest++ = *source++;
    ctr++;
  }
  *dest = 0; // Mark end of string
  return ctr;
}

void lcase(char *string) {
  while (*string != 0) {
    if (*string >= 65 && *string <= 90) *string += 32;
    string++;
  }
}

char isnumeric(char *string) {
  char *pos;
  if (*string == 0) { // Blank
    return 0;
  }
  pos = string;
  while (*pos != 0) {
    if ((*pos < 48) || (*pos > 57)) {
      return 0;
    }
    pos++;
  }
  return 1;
}

static unsigned int StrToNum(char *input, char maxdigits) {
  char b;
  char *pos;
  unsigned int output;
  output=0;
  pos = input;
  for (b=0;b<maxdigits;b++) {
    if (*pos >= '0' && *pos <='9') {
      output *= 10;
      output += (*pos - 48);
    } else if (*pos == 0) {
      b = 250; //End of string, exit loop
    }
    pos++;
  }
  return output;
}

char StrToByte(char *input) {
  /*char b;
  char *pos;
  char output;
  output=0;
  pos = input;
  for (b=0;b<3;b++) {
    if (*pos >= '0' && *pos <='9') {
      output *= 10;
      output += (*pos - 48);
    } else if (*pos == 0) {
      b = 250; //End of string, exit loop
    }
    pos++;
  }
  return output;*/
  return StrToNum(input, 3);
}

unsigned int StrToWord(char *input) {
  return StrToNum(input, 5);
}

char *RomTxt30(const char *ctxt) {
  static char txt[30];
  char b;
  for (b=0;b<30;b++) {
    txt[b] = ctxt[b];
    if (txt[b] == 0) {b = 250;} // Exit for
  }
  //for (b=0;txt[b]=ctxt[b];b++);
  return txt;
}