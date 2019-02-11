package main

import (
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"sort"
)

func main () {
	path := "./pics"
	files, err := ioutil.ReadDir(path)
	if err != nil {
		log.Fatal(err)
	}
	names := make([]string, 0)
	for _, f := range files {
		names = append(names, f.Name())
	}
	sort.Slice(names, func(i, j int) bool {return names[i] < names[j]})
	size := len(names)
	for i := 0; i < size; i += 2 {
		ID := fmt.Sprintf("%04d.jpg", i / 2)
		err := os.Rename(path + "/" + names[i], "./masks/" + ID)
		if err != nil {
			log.Fatal(err)
		}
		err = os.Rename(path + "/" + names[i + 1], "./raws/" + ID)
		if err != nil {
			log.Fatal(err)
		}
	}
}
