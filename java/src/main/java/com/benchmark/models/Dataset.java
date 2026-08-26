package com.benchmark.models;

import java.util.List;
import java.util.Map;

public class Dataset {
    public Map<String, Object> metadata;
    public Vehicle vehicle;
    public Config configuration;
    public List<Node> nodes;
    public List<Edge> edges;
    public List<Query> queries;

    public Dataset() {}
}
