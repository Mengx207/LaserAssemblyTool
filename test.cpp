using namespace std;
 
int main()
{
    // Opening the file
    ifstream file("file.txt");
 
    vector<string> v;
    string str;
 
    // Read the next line from File until it reaches the
    // end.
    while (file >> str) {
        // Now keep reading next line
        // and push it in vector function until end of file
        v.push_back(str);
    }
 
    // Printing each new line separately
    copy(v.begin(), v.end(),
         ostream_iterator<string>(cout, "\n"));
 
    return 0;
}