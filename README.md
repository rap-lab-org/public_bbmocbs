# Binary Branching Multi-Objective Conflict-Based Search (BB-MO-CBS)

This repository provides C++ Implementation of the Binary Branching Multi-Objective Conflict-Based Search (BB-MO-CBS) algorithm [1] for Multi-Objective Multi-Agent Path Finding problem. This project is still under active development. 

BB-MO-CBS is a new algorithm that outperforms our prior MO-CBS [2].

The code is distributed for academic and non-commercial use.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Requirements

* We use CMake (3.16.3) and Make (4.2.1) on Ubuntu 20.04 to compile the code. Lower or higher version may also work. 
* We use C++11. Lower or higher version of C++ may also work.
* We massively use C++ STL, which is very common in C++ programming nowadays. We do not use any third-party libraries, and this package should be straightfoward to compile by following the instructions below.

## Project Structure

* `README.md` - This file
* `cpp/source/` - Contains the path planning source code
* `cpp/test/` - Contains example code for MO-CBS algorithms
* `cpp/include/` - Contains header files

## Instructions

* Clone this repo
* Compile this repo
  * `mkdir build`
  * `cd build`
  * `cmake ..` (You can specify the build type you like by adding additional args)
  * `make`
* Run example in `./test_bb_mocbs `
* This example shows how to use the code.

## References

Please cite the corresponding papers if you find anything in this repository helpful.

* [1] TODO

* [2] A Conflict-Based Search Framework for Multi-Objective Multi-Agent Path Finding\
	**Zhongqiang Ren**, Sivakumar Rathinam, and Howie Choset.\
	<i>IEEE Transactions on Automation Science and Engineering (T-ASE)</i>, 2022.\
[[Bibtex](https://wonderren.github.io/files/bibtex_ren21mocbs_tase.txt)]
[[Paper](https://wonderren.github.io/files/ren22_mocbs_tase_final.pdf)]
[[Talk](https://youtu.be/KI-BVhsjg0I)]
